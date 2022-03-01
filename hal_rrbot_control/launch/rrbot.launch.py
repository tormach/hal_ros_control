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
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller_config",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ros2_control_demo_bringup"),
                    "config",
                    "rrbot_controllers.yaml",
                ]
            ),
            description="YAML file with the controllers configuration.",
        )
    )

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
            "description_package",
            default_value="rrbot_description",
            description="Description package with robot URDF/xacro files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="rrbot.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="false",
            description="Enable Gazebo in URDF.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "with_sensor",
            default_value="false",
            description="Enable integrated sensor.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Start robot in Gazebo simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "position_only",
            default_value="true",
            description="Do not configure velocity or acceleration interfaces.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description=(
                "Start robot with fake hardware mirroring command to state."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "slowdown",
            default_value="3.0",
            description="Slowdown factor of the RRbot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="forward_position_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(  # Changed for hal_rrbot_control demo
        DeclareLaunchArgument(
            "hardware_plugin",
            default_value="hal_system_interface/HalSystemInterface",
            description="ros2_control hardware plugin.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # hal_hw_interface launch args
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

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(
                        LaunchConfiguration("description_package")
                    ),
                    "urdf",
                    LaunchConfiguration("description_file"),
                ]
            ),
            " ",
            "prefix:=",
            LaunchConfiguration("prefix"),
            " ",
            "use_gazebo:=",
            LaunchConfiguration("use_gazebo"),
            " ",
            "use_sim:=",
            LaunchConfiguration("use_sim"),
            " ",
            "use_fake_hardware:=",
            LaunchConfiguration("use_fake_hardware"),
            " ",
            "with_sensor:=",
            LaunchConfiguration("with_sensor"),
            " ",
            "position_only:=",
            LaunchConfiguration("position_only"),
            " ",
            "fake_sensor_commands:=",
            LaunchConfiguration("fake_sensor_commands"),
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
                    dict(
                        use_sim=LaunchConfiguration("use_sim"),
                    ),
                ],
            ),
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare(LaunchConfiguration("description_package")),
            "config",
            "rrbot.rviz",
        ]
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
        executable="spawner.py",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
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
