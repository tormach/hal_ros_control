import os
import subprocess
from machinekit import config
from .ros_hal_component import RosHalComponent

# ROS
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory


class HalMgr(RosHalComponent):
    compname = "hal_mgr"
    READY_TOPIC = "hal_mgr/ready"

    def init_hal_comp(self):
        # Don't actually set up any HAL comp
        pass

    def setup_component(self):
        # Not actually setting up HAL comp; don't call
        # super().setup_component()
        self.init_ready_topic_publisher()
        self.start_realtime()

    def init_ready_topic_publisher(self):
        """Set up "ready" status publisher."""
        self.ready_pub = self.node.create_publisher(Bool, self.READY_TOPIC, 1)
        self.ready_msg = Bool()
        self.publish_ready_topic(value=False)

    def start_realtime(self):
        """
        Start new RTAPI/HAL session.

        Start Machinekit `realtime` environment.  Register the
        :py:func:`stop_realtime` shutdown callback.
        """
        # Set up console debug output and config
        d_out = "1" if self.get_ros_param("hal_debug_output", True) else "0"
        d_lev = str(self.get_ros_param("hal_debug_level", 1))
        # - Use UDP discovery; rtapi_app runs as root, which breaks
        #   shm-based discovery
        #   https://github.com/eProsima/Fast-DDS/issues/1750
        fastrtps_profiles = os.path.join(
            get_package_share_directory("hal_hw_interface"),
            "config",
            "fastrtps_disable_shm.xml",
        )
        self.logger.info(f"Applying fastRTPS SHM hack:  {fastrtps_profiles}")

        self.logger.info(f"Starting hal_mgr; debug={d_out}/level={d_lev}")
        env = dict(
            DEBUG=d_lev,
            SYSLOG_TO_STDERR=d_out,
            MACHINEKIT_INI=config.Config().MACHINEKIT_INI,
            FASTRTPS_DEFAULT_PROFILES_FILE=fastrtps_profiles,
            **os.environ,
        )
        subprocess.check_call(["realtime", "start"], env=env)
        self.add_shutdown_callback(self.stop_realtime, 999)  # run last

        # Signal readiness
        self.publish_ready_topic(value=True)
        self.logger.info("hal_mgr:  Started realtime")

    def stop_realtime(self):
        self.logger.warn("Stopping realtime")
        try:
            subprocess.check_call(["halcmd", "stop"])
            subprocess.check_call(["halcmd", "unloadrt", "all"])
            subprocess.check_call(["halcmd", "unload", "all"])
        finally:
            subprocess.check_call(["realtime", "stop"])
            self.logger.info("Realtime stopped")

    def update(self):
        # FIXME Watch realtime; shut down if it shuts down
        self.publish_ready_topic()

    def publish_ready_topic(self, value=None):
        """
        Publish the HAL config "ready" status.

        This topic coming `True` indicates RTAPI/HAL is running.

        Change published value by setting the `value` param to `True`
        or `False.
        """
        if value is not None:
            if self.ready_msg.data != value:
                self.logger.info(f"hal_mgr/ready topic new value:  {value}")
            self.ready_msg.data = value
        self.ready_pub.publish(self.ready_msg)
