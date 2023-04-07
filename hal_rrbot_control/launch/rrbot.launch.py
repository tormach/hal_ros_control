# Copyright 2021 John Morris john@zultron.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Much cribbed from ros2_control_demo_bringup package,
# launch/rrbot_base.launch.py at rev 864b924
# https://github.com/ros-controls/ros2_control_demos

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from hal_hw_interface.launch import HalConfig, HalRTNode, HalFiles


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    # - Find ros2_control_demos example package
    declared_arguments.append(
        DeclareLaunchArgument(
            "ros2_control_demo_example_num",
            default_value="1",
            description="ros2_control_demos example number (1-8).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ros2_control_demo_example_package",
            default_value=[
                "ros2_control_demo_example_",
                LaunchConfiguration("ros2_control_demo_example_num"),
            ],
            description="ros2_control_demos example package name.",
        )
    )
    example_package_share = FindPackageShare(
        LaunchConfiguration("ros2_control_demo_example_package")
    )

    # - Find ros2_control_demos example package config
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller_config",
            default_value=PathJoinSubstitution(
                [
                    example_package_share,
                    "config",
                    "rrbot_controllers.yaml",
                ]
            ),
            description="YAML file with the controllers configuration.",
        )
    )

    # - HAL hardware interface & controller
    declared_arguments.append(
        DeclareLaunchArgument(
            "hal_file_dir",
            default_value=PathJoinSubstitution(
                [FindPackageShare("hal_rrbot_control"), "halfiles"]
            ),
            description="Directory containing HAL configuration files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hal_file",
            default_value="rrbot.hal",
            description="HAL file to load from hal_file_dir.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="forward_position_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hal_debug_output",
            default_value="true",
            description="Output HAL debug messages to screen (console).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hal_debug_level",
            default_value="1",
            description="Set HAL debug level, 0-5.",
        )
    )

    # - URDF
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            # FIXME Varies with each example
            # example_1/description/urdf/rrbot.urdf.xacro
            # example_1/description/urdf/rrbot_description.urdf.xacro (macro)
            # example_2/description/urdf/diffbot.urdf.xacro
            # example_2/description/urdf/diffbot_description.urdf.xacro (macro)
            # example_3/description/urdf/rrbot_system_multi_interface.urdf.xacro
            # example_3/description/urdf/rrbot_description.urdf.xacro (macro)
            # example_4/description/urdf/rrbot_system_with_sensor.urdf.xacro
            # example_4/description/urdf/rrbot_description.urdf.xacro  (macro)
            # example_5/description/urdf/rrbot_system_with_external_sensor.urdf.xacro
            # example_5/description/urdf/rrbot_description.urdf.xacro (macro)
            # example_6/description/urdf/rrbot_modular_actuators.urdf.xacro
            # example_6/description/urdf/rrbot_description.urdf.xacro (macro)
            # example_8/description/urdf/rrbot_transmissions_system_position_only.urdf.xacro
            # example_8/description/urdf/rrbot_description.urdf.xacro (macro)
            default_value="rrbot.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description=(
                "Start robot with mock hardware mirroring command to state."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "slowdown",
            default_value="3.0",
            description="Slowdown factor of the RRbot.",
        )
    )
    declared_arguments.append(  # Changed for hal_rrbot_control demo
        DeclareLaunchArgument(
            "hardware_plugin",
            default_value="hal_system_interface/HalSystemInterface",
            description="ros2_control hardware plugin.",
        )
    )

    # - RViz
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    rviz_config_file = PathJoinSubstitution(
        [
            example_package_share,
            "rviz",
            "rrbot.rviz",
        ]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    example_package_share,
                    "urdf",
                    LaunchConfiguration("description_file"),
                ]
            ),
            " ",
            "use_mock_hardware:=",
            LaunchConfiguration("use_mock_hardware"),
            " ",
            "mock_sensor_commands:=",
            LaunchConfiguration("mock_sensor_commands"),
            " ",
            "slowdown:=",
            LaunchConfiguration("slowdown"),
            " ",
            "hardware_plugin:=",
            LaunchConfiguration("hardware_plugin"),
        ]
    )

    # HAL configuration
    hal_hw_interface_launch = HalConfig(
        # hal_mgr args
        parameters=[
            dict(  # Individual keys
                hal_debug_output=LaunchConfiguration("hal_debug_output"),
                hal_debug_level=LaunchConfiguration("hal_debug_level"),
            ),
        ],
        output="both",
        emulate_tty=True,
        log_cmd=True,
        # HAL config
        actions=[
            # Hardware interface
            HalRTNode(
                package="hal_hw_interface",
                component="hal_control_node",
                parameters=[
                    dict(robot_description=robot_description_content),
                    LaunchConfiguration("robot_controller_config"),
                ],
                log_cmd=True,
                output="both",
                emulate_tty=True,
                # arguments=['--ros-args', '--log-level', 'debug'],
                # prefix=['xterm -e gdb -ex run --args'],
            ),
            # HAL files
            HalFiles(
                hal_file_dir=LaunchConfiguration("hal_file_dir"),
                hal_files=[LaunchConfiguration("hal_file")],
                parameters=[
                    # For Python HAL files, add Node-style parameters here
                ],
            ),
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[dict(robot_description=robot_description_content)],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            LaunchConfiguration("robot_controller"),
            "-c",
            "/controller_manager",
        ],
    )

    nodes = [
        hal_hw_interface_launch,
        robot_state_pub_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
