import os
import subprocess
from machinekit import config, rtapi
from .ros_hal_component import RosHalComponent

# ROS
from std_msgs.msg import Bool


class HalMgr(RosHalComponent):
    compname = "hal_mgr"
    READY_TOPIC = "hal_mgr/ready"
    shutdown_begun = False
    update_rate = 0.1  # sec

    def init_hal_comp(self):
        # Don't actually set up any HAL comp
        pass

    def setup_component(self):
        # Not actually setting up HAL comp; don't call
        # super().setup_component()
        self.init_ready_topic_publisher()
        self.start_realtime()
        self.load_hal_config()

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
        os.environ.setdefault("DEBUG", "0")
        if "MACHINEKIT_INI" not in os.environ:  # export for package installs
            mkconfig = config.Config()
            os.environ["MACHINEKIT_INI"] = mkconfig.MACHINEKIT_INI
        subprocess.check_call("realtime start", shell=True)
        self.add_shutdown_callback(self.stop_realtime, 999)  # run last
        self.logger.info("hal_mgr:  Started realtime")

    def stop_realtime(self):
        self.logger.warn("Stopping realtime")
        subprocess.check_call("realtime stop", shell=True)
        self.logger.info("Realtime stopped")

    def load_hal_config(self):
        # Be sure rtapi is initialized (used in python HAL configs)
        if not getattr(rtapi, "__rtapicmd") or not rtapi.__rtapicmd:
            rtapi.init_RTAPI()

        # Load the hal_hw_interface
        hal_comp_dir = self.get_ros_param("hal_comp_dir")
        assert hal_comp_dir
        hal_hw_interface_path = os.path.join(hal_comp_dir, "hal_control_node")
        self.logger.info(f"hal_hw_interface_path:  {hal_hw_interface_path}")
        rtapi.loadrt(hal_hw_interface_path, ARGV=",".join(self.argv))
        self.logger.info("rosctl_intf comp loaded successfully")

        # Get HAL config directory and filename list from ROS parameters
        hal_file_dir = self.get_ros_param("hal_file_dir")
        assert hal_file_dir is not None
        hal_files = self.get_ros_param("hal_files")
        assert isinstance(hal_files, list)

        # Munge directory and filenames into paths
        hal_file_paths = list()
        for hal_file in hal_files:
            hal_file_path = os.path.join(hal_file_dir, hal_file)
            assert os.path.exists(hal_file_path)
            hal_file_paths.append(hal_file_path)

        # Load HAL configuration from HAL files
        for fpath in hal_file_paths:
            self.logger.info(f"Loading hal file '{fpath}'")
            if fpath.endswith(".py"):
                with open(fpath, "r") as f:
                    data = compile(f.read(), fpath, "exec")
                globals_ = {}
                exec(data, globals_)
            else:
                subprocess.check_call(["halcmd", "-f", fpath])

        # Signal readiness
        self.publish_ready_topic(value=True)
        self.logger.info("HAL configuration loaded")

    def update(self):
        self.publish_ready_topic()

    def publish_ready_topic(self, value=None):
        """
        Publish the HAL config "ready" status.

        This topic coming `True` indicates the HAL configuration is
        complete.

        Change published value by setting the `value` param to `True`
        or `False.
        """
        if value is not None:
            self.ready_msg.data = value
        self.ready_pub.publish(self.ready_msg)
