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

    # Find the hal_hw_interface comp's directory in LD_LIBRARY_PATH and put it
    # into $COMP_DIR
    comp_dir = ""
    for path in os.environ.get('LD_LIBRARY_PATH','').split(':'):
        if os.path.exists(os.path.join(path, 'hal_hw_interface.so')):
            comp_dir = path
    os.environ['COMP_DIR'] = comp_dir
    rospy.loginfo("hal_mgr:  COMP_DIR set to '%s'" % comp_dir)

    # Get parameters
    if not rospy.has_param(NAME):
        shutdown("No parameters set for '%s'" % NAME, 1)
    try:
        hal_mgr_config = rospy.get_param(NAME)
    except KeyError:
        shutdown("No keys defined at %s" % NAME, 1)
    if 'hal_files' not in hal_mgr_config:
        shutdown("%s has no 'hal_files' key" % NAME, 1)
    if 'hal_file_dir' not in hal_mgr_config:
        shutdown("%s has no 'hal_file_dir' key" % NAME, 1)

    # Set up HAL
    launcher.cleanup_session()  # kill any running Machinekit instances
    launcher.start_realtime()
    if not rtapi.__rtapicmd:
        rtapi.init_RTAPI()
    rospy.loginfo("hal_mgr:  Started realtime")

    # Load HAL configuration
    robot_hw_loaded = False
    hal_io_loaded = False
    os.environ['PATH'] += (":%s" % SCRIPT_DIR) # Help HAL find hal_io comp
    for fname in hal_mgr_config['hal_files']:
        fpath = os.path.join(hal_mgr_config['hal_file_dir'], fname)
        if not os.path.exists(fpath):
            shutdown("No file '%s' in directory '%s'" %
                     (fname, hal_mgr_config['hal_file_dir']))
        rospy.loginfo("hal_mgr:  Loading hal file '%s'" % fname)
        launcher.load_hal_file(fpath)
        rospy.loginfo("hal_mgr:  Loading hal file '%s' complete" %
                      fpath)

    # Spin until ROS shutdown event
    rospy.loginfo("ROS node and HAL started successfully")
    while not rospy.is_shutdown():
        launcher.check_processes()
        rate.sleep()

except subprocess.CalledProcessError as e:
    shutdown("Process error:  %s" % e, 1)
except rospy.ROSInterruptException as e:
    shutdown("Interrupt:  %s" % e, 0)

shutdown("Shutting down", 0)
