# ROS HAL interface

This package provides interfaces to [Machinekit][machinekit] HAL from
ROS.  It is expected to realize these benefits:

- Bring real-time control to ROS with Machinekit RTAPI
- Run on inexpensive PC and ARM hardware
- Enable setting up many robot hardware interfaces with configuration
  only, no programming
- Leverage Machinekit's wide variety of real-time components:
  - Hardware drivers:  EtherCAT, BeagleBone GPIO, Mesa Electronics
    AnythingI/O FPGA cards, PC parallel port
  - Motor control:  quadrature encoders, step/direction, PWM
    generators, PID controllers
  - Arithmetic:  limit position/velocity/acceleration, scale/offset,
    low-pass filter, derivative/integral, multiplexer
  - Logic:  and/or/xor/not, look-up tables, debouncer, flip-flop
  - And many more, plus simple tools to write custom components in C

This package provides two main HAL components for interfacing with
ROS, plus infrastructure to configure and run the system.  The
`hal_hw_interface` HAL RT component enables robot joint control by
extending [`ros2_control`][ros2_control].  The `hal_io` user component
(non-real-time) enables simple robot I/O by connecting HAL input and
output pins with `std_msgs` publishers and subscribers, respectively.

[machinekit]:  http://machinekit.io
[ros2_control]: https://github.com/ros-controls/ros2_control

## The `hal_hw_interface` real-time component

The `hal_control_node` HAL component runs a
`controller_manager::ControllerManager` node in a real-time HAL
thread, and the complementary
`hal_system_interface::HalSystemInterface` is a
`hardware_interface::SystemInterface` implementation that creates HAL
pins for the various state and command interfaces defined in a robot's
`ros2_control` URDF.

## `hal_io`

The `hal_io` HAL user (non-realtime) component publishes and
subscribes to ROS `std_msgs` topics with values read from and written
to robot I/O HAL pins.  The `HAL_BIT`, `HAL_FLOAT`, `HAL_S32` and
`HAL_U32` HAL pin value types correspond to `Bool`, `Float64`, `Int32`
and `UInt32` messages from ROS `std_msgs`.

## `hal_hw_interface.launch`:  ROS2 launch actions for HAL

The `hal_hw_interface.launch` Python module makes it easy to launch a
robot with HAL hardware.  It provides actions to start the Machinekit
HAL realtime environment, load the `hal_control_node`, `hal_io` and
any other HAL components running ROS2 Nodes, and load HAL files.  It
takes care of executing these actions in sequence so the HAL
components don't start before HAL is running, and HAL files don't
start netting pins before the HAL components are loaded and ready.

## Dependencies

- [Machinekit][machinekit]
  - Required by all components
  - Install Machinekit from packages or
    source on a PC by following instructions on that site.
- A real-time kernel, either RT_PREEMPT or Xenomai
  - Required by Machinekit for low-latency control
  - See the `linux-image-rt-*` packages available in Debian Stretch.
- [`ros_control_boilerplate`][ros_control_boilerplate]
  - Required by the `hal_hw_interface`
  - This may be installed in package form
- Optional:  The `ros2_control_demos` repo
  - Required to run the `hal_rrbot_control` demo
  - Follow the notes in that project's README to install

-----
## Run the demos

This demo runs the `ros2_control_demos` "RRBot" two joint
revolute-revolute robot demo with HAL.  It is meant to show how simple
it can be to build a `ros_control` hardware interface with HAL, and to
serve as an example for creating your own.

Run the simulated hardware interface:

    ros2 launch hal_rrbot_control rrbot.launch.py
    # Debugging: append `hal_debug_output:=1 hal_debug_level:=5`
    # Trajectory controller:  append
    #     `robot_controller:=position_trajectory_controller`

Run `halscope` to visualize HAL joint commands and feedback; in the
GUI, set the "Run Mode" to "Roll" for continuous updating:

    halscope -i hal_rrbot_control/config/hal_hw_interface.halscope

The simulated trajectories are launched directly from the
`ros2_control_demo_bringup` package:

    ros2 launch ros2_control_demo_bringup test_forward_position_controller.launch.py
    # Or for robot_controller:=position_trajectory_controller:
    ros2 launch ros2_control_demo_bringup test_joint_trajectory_controller.launch.py

Load and switch to the trajectory controller at run time:

    # Load and configure the trajectory controller
    ros2 service call \
      /controller_manager/load_and_configure_controller \
      controller_manager_msgs/srv/LoadConfigureController \
      '{name: "position_trajectory_controller"}'

    # Switch to the trajectory controller
    ros2 service call \
      /controller_manager/switch_controller \
      controller_manager_msgs/srv/SwitchController \
      '{start_controllers: ["position_trajectory_controller"],
        stop_controllers: ["forward_position_controller"],
        strictness: 1, start_asap: true,
        timeout: {sec: 0, nanosec: 10000000}
       }'

    # Verify the switch: 'position_trajectory_controller' state='active'
    ros2 service call \
      /controller_manager/list_controllers \
      controller_manager_msgs/srv/ListControllers

-----
## Configuration

The HAL configuration is started from and configured primarily through
the robot's launch configuration.  See the working example in
`hal_rrbot_control/launch/rrbot.launch.py`.

In general, a ROS2 launch HAL configuration includes a
`hal_hw_interface.launch.HalConfig` group action that starts the
`hal_mgr` Node, and contains a group of actions including a
`hal_hw_interface.launch.HalRTNode` for launching the
`hal_control_node`, one or more `hal_hw_interface.launch.HalUserNode`
for launching `hal_io` or other HAL user component Nodes, followed by
a `hal_hw_interface.launch.HalFiles` action to load one or more Python
or `.hal` HAL files and complete the configuration.

The robot description's URDF must contain something like the
following:

    <ros2_control name="${name}" type="system">
      <hardware>
        <plugin>hal_system_interface/HalSystemInterface</plugin>
      </hardware>
      <!-- joints... -->
    </ros2_control>

This tells the controller manager to load the HAL system interface
plugin for this robot; this plugin will create HAL pins for each state
or command interface.

Tests:
```
colcon test \
    --event-handlers console_cohesion+ \
    --executor sequential
colcon test-result --verbose
```

## Acknowledements

This work was sponsored by [Tormach][tormach] for use in its ZA6 6DOF
robot arm.

The original ROS(1) C++ HAL integration was done following examples by
Bas de Bruijn and Mick Grant of the [Machinekit project][machinekit].
The `hal_mgr` ROS node, which starts up the RTAPI and HAL apparatus,
is based on Alexander Roessler's [`python-hal-seed`][python-hal-seed].
The control loop was inspired by the
[`rtt_ros_control_example`][rtt_ros_control_example] and related
[discussion][ros_control-130].

[tormach]:  https://www.tormach.com/
[python-hal-seed]: https://github.com/machinekoder/python-hal-seed
[rtt_ros_control_example]: https://github.com/skohlbr/rtt_ros_control_example
[ros_control-130]: https://github.com/ros-controls/ros_control/issues/130
