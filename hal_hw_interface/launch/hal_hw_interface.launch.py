from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "hal_file_dir",
            description="Directory containing HAL configuration files.",
        ),
        DeclareLaunchArgument(
            "hal_files",
            description="List of HAL files to load from hal_file_dir.",
        ),
        DeclareLaunchArgument(
            "robot_description",
            description="Robot description from URDF.",
        ),
        DeclareLaunchArgument(
            "robot_controllers_param_file",
            description="Robot controller configuration param file.",
        ),
        DeclareLaunchArgument(
            "hal_debug_output",
            default_value="false",
            description="Output HAL debug messages to screen (console).",
        ),
        DeclareLaunchArgument(
            "hal_debug_level",
            default_value="1",
            description="Set HAL debug level, 0-5.",
        ),
    ]

    # ROS parameters from various sources
    params = [
        # Individual keys
        dict(
            hal_file_dir=LaunchConfiguration("hal_file_dir"),
            hal_files=LaunchConfiguration("hal_files"),
            robot_description=LaunchConfiguration("robot_description"),
        ),
        # Generated yaml with build-time configuration
        PathJoinSubstitution(
            [
                FindPackageShare("hal_hw_interface"),
                "config",
                "hal_hw_interface.yaml",
            ]
        ),
        # Robot controllers yaml from launch params
        LaunchConfiguration("robot_controllers_param_file"),
    ]

    # Pass in HAL debug settings via environment
    hal_debug_output = LaunchConfiguration("hal_debug_output")
    hal_debug_level = LaunchConfiguration("hal_debug_level")
    hal_debug_output_exp = PythonExpression(
        ["'1' if '", hal_debug_output, "' == 'true' else ''"]
    )
    fastrtps_disable_shm = PathJoinSubstitution(
        [
            # Use UDP discovery; rtapi_app runs as root, which breaks
            # shm-based discovery
            # https://github.com/eProsima/Fast-DDS/issues/1750
            FindPackageShare("hal_hw_interface"),
            "config",
            "fastrtps_disable_shm.xml",
        ]
    )

    node_env = dict(
        SYSLOG_TO_STDERR=hal_debug_output_exp,
        DEBUG=hal_debug_level,
        FASTRTPS_DEFAULT_PROFILES_FILE=fastrtps_disable_shm,
    )

    # Run the node
    hal_mgr_node = Node(
        package="hal_hw_interface",
        executable="hal_mgr",
        parameters=params,
        additional_env=node_env,
        output="both",
        emulate_tty=True,
        # arguments=['--ros-args', '--log-level', 'debug'],
        # prefix=['xterm -e gdb -ex run --args'],
    )

    return LaunchDescription(declared_arguments + [hal_mgr_node])
