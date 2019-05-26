# -*- coding: utf-8 -*-
import sys
import os
import subprocess
import signal

from machinekit import launcher, config, rtapi

# ROS
import rospy
from std_msgs.msg import Bool


class HalMgr(object):
    NAME = 'hal_mgr'
    READY_TOPIC = "hal_mgr/ready"
    shutdown_begun = False

    def __init__(self):
        # Set up ROS node
        rospy.init_node(self.NAME)
        # Call end_session() on ROS shutdown; don't have the launcher
        # register its own exit handler
        rospy.on_shutdown(lambda: self.shutdown('Graceful shutdown via ROS'))
        self._rate = rospy.Rate(1)  # 1hz
        rospy.loginfo("hal_mgr:  Initialized node")

        self._pub = rospy.Publisher(
            self.READY_TOPIC, Bool, queue_size=1, latch=True
        )

    def start(self):
        # Find the hal_hw_interface comp's directory in LD_LIBRARY_PATH and put it
        # into $COMP_DIR
        comp_dir = ""
        for path in os.environ.get('LD_LIBRARY_PATH', '').split(':'):
            if os.path.exists(os.path.join(path, 'hal_hw_interface.so')):
                comp_dir = path
        os.environ['COMP_DIR'] = comp_dir
        rospy.loginfo("hal_mgr:  COMP_DIR set to '%s'" % comp_dir)

        # Get parameters
        if not rospy.has_param(self.NAME):
            self.shutdown("No parameters set for '%s'" % self.NAME, 1)
        try:
            hal_mgr_config = rospy.get_param(self.NAME)
        except KeyError:
            self.shutdown("No keys defined at %s" % self.NAME, 1)
            return

        if 'hal_files' not in hal_mgr_config:
            self.shutdown("%s has no 'hal_files' key" % self.NAME, 1)
        if 'hal_file_dir' not in hal_mgr_config:
            self.shutdown("%s has no 'hal_file_dir' key" % self.NAME, 1)

        # Set up HAL
        launcher.cleanup_session()  # kill any running Machinekit instances
        launcher.start_realtime()
        rospy.loginfo("hal_mgr:  Started realtime")

        # Load rtapi module and set up signal handlers
        if not getattr(rtapi, '__rtapicmd'):
            rtapi.init_RTAPI()

        def shutdown_graceful(signum, frame):
            self.shutdown('Gracefully shutting down after interrupt signal')

        signal.signal(signal.SIGINT, shutdown_graceful)
        signal.signal(signal.SIGTERM, shutdown_graceful)

        # Load HAL configuration
        for fname in hal_mgr_config['hal_files']:
            fpath = os.path.join(hal_mgr_config['hal_file_dir'], fname)
            if not os.path.exists(fpath):
                self.shutdown(
                    "No file '%s' in directory '%s'"
                    % (fname, hal_mgr_config['hal_file_dir'])
                )
            rospy.loginfo("hal_mgr:  Loading hal file '%s'" % fname)
            launcher.load_hal_file(fpath)
            rospy.loginfo("hal_mgr:  Loading hal file '%s' complete" % fpath)

        # Spin until ROS shutdown event
        rospy.loginfo("ROS node and HAL started successfully")
        self._pub.publish(True)

    def loop(self):
        while not rospy.is_shutdown():
            launcher.check_processes()
            self._rate.sleep()

    def shutdown(self, msg="Shutting down for unknown reason", res=0):
        # Only run this once
        if self.shutdown_begun:
            return
        self.shutdown_begun = True

        if res:
            rospy.logerr("hal_mgr:  %s" % msg)
        else:
            rospy.loginfo("hal_mgr:  %s" % msg)
        self._pub.publish(False)
        launcher.end_session()
        rospy.signal_shutdown(msg)
        sys.exit(res)


def main():
    debug = int(os.environ.get('DEBUG', 0))
    launcher.set_debug_level(debug)

    if 'MACHINEKIT_INI' not in os.environ:  # export for package installs
        mkconfig = config.Config()
        os.environ['MACHINEKIT_INI'] = mkconfig.MACHINEKIT_INI

    hal_mgr = HalMgr()
    try:
        hal_mgr.start()
        hal_mgr.loop()
    except subprocess.CalledProcessError as e:
        hal_mgr.shutdown("Process error:  %s" % e, 1)
    except rospy.ROSInterruptException as e:
        hal_mgr.shutdown("Interrupt:  %s" % e, 0)
    else:
        hal_mgr.shutdown("Shutting down", 0)
