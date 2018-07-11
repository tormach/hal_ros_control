#!/usr/bin/env python

import sys
import os
import subprocess
from machinekit import launcher, config, rtapi, hal

# ROS
import rospy

MAIN_HAL = 'main.py'
NAME = 'hal_mgr'
HAL_IO = 'hal_io'
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))

os.chdir(SCRIPT_DIR)

debug = int(os.environ.get('DEBUG',0))
launcher.set_debug_level(debug)

def shutdown(msg = "Shutting down for unknown reason", res = 0):
    launcher.end_session()
    if res:
        rospy.logerr("hal_mgr:  %s" % msg)
    else:
        rospy.loginfo("hal_mgr:  %s" % msg)
    rospy.signal_shutdown(msg)
    sys.exit(res)

if 'MACHINEKIT_INI' not in os.environ:  # export for package installs
    mkconfig = config.Config()
    os.environ['MACHINEKIT_INI'] = mkconfig.MACHINEKIT_INI

try:
    # Set up ROS node
    rospy.init_node(NAME, anonymous=True)
    # Call end_session() on ROS shutdown; don't have the launcher
    # register its own exit handler
    rospy.on_shutdown(lambda: shutdown('Graceful shutdown via ROS'))
    rate = rospy.Rate(1) # 1hz
    rospy.loginfo("hal_mgr:  Initialized node")

    # Get parameters
    if not rospy.has_param(NAME):
        shutdown("No parameters set for '%s'" % NAME, 1)
    try:
        hal_mgr_config = rospy.get_param(NAME)
    except KeyError:
        shutdown("No keys defined at %s" % NAME, 1)
    if 'hal_config' not in hal_mgr_config:
        shutdown("%s has no 'hal_config' key" % NAME, 1)
    if 'hal_file_dir' not in hal_mgr_config:
        shutdown("%s has no 'hal_file_dir' key" % NAME, 1)
    if 'hal_comp' not in hal_mgr_config:
        shutdown("%s has no 'hal_comp' key" % NAME, 1)

    # Set up HAL
    launcher.cleanup_session()  # kill any running Machinekit instances
    launcher.start_realtime()
    if not rtapi.__rtapicmd:
        rtapi.init_RTAPI()
    rospy.loginfo("hal_mgr:  Started realtime")

    # Load HAL configuration
    robot_hw_loaded = False
    hal_io_loaded = False
    for a in hal_mgr_config['hal_config']:
        if 'cmd' not in a:
            shutdown("%s entry has no 'cmd' key" % NAME, 1)
        if a['cmd'] == 'load_hal_file':
            if 'fname' not in a:
                shutdown(
                    "%s 'load_hal_file' command has no 'fname' key" % NAME, 1)
            rospy.loginfo("hal_mgr:  Loading hal file '%s'" % a['fname'])
            launcher.load_hal_file(
                os.path.join(hal_mgr_config['hal_file_dir'], a['fname']))
            rospy.loginfo("hal_mgr:  Loading hal file '%s' complete" %
                          a['fname'])
        elif a['cmd'] == 'load_robot_hw':
            if robot_hw_loaded:
                shutdown(
                    "%s duplicate 'load_robot_hw' command" % NAME, 1)
            robot_hw_loaded = True
            if 'thread' not in a:
                shutdown(
                    "%s 'load_robot_hw' command has no 'thread' key" % NAME, 1)
            if "ROS_MASTER_URI" in os.environ:
                # This is probably the wrong way to set the master URI...
                os.environ["ROS_MASTER_URI"] = (
                    "http://%s/" % rospy.get_master().target._ServerProxy__host)
            rospy.loginfo(
                "hal_mgr:  Loading RT comp '%s', ROS_MASTER_URI '%s'" %
                (hal_mgr_config['hal_comp'], os.environ["ROS_MASTER_URI"]))
            comp = rtapi.loadrt(hal_mgr_config['hal_comp'])
            rospy.loginfo("hal_mgr:  Adding RT function to thread '%s'" %
                          a['thread'])
            hal.addf("%s.funct" % comp.name, a['thread'])
            rospy.loginfo("hal_mgr:  Loading RT comp '%s' complete" %
                          hal_mgr_config['hal_comp'])
        elif a['cmd'] == 'load_hal_io':
            if hal_io_loaded:
                shutdown(
                    "%s duplicate 'load_hal_io' command" % NAME, 1)
            hal_io_loaded = True
            hal_io_path = os.path.join(SCRIPT_DIR, HAL_IO)
            rospy.loginfo("hal_mgr:  Loading user comp '%s'" %
                          hal_io_path)
            from pprint import pformat
            comp = hal.loadusr(hal_io_path, wait_name=HAL_IO)
            rospy.loginfo("hal_mgr:  Loading user comp '%s' complete" %
                          HAL_IO)
        else:
            shutdown("%s invalid command '%s'" % (NAME, a['cmd']), 1)

    rospy.loginfo("ROS node and HAL started successfully")
    while not rospy.is_shutdown():
        launcher.check_processes()
        rate.sleep()

except subprocess.CalledProcessError as e:
    shutdown("Process error:  %s" % e, 1)
except rospy.ROSInterruptException as e:
    shutdown("Interrupt:  %s" % e, 0)

shutdown("Shutting down", 0)
