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

The `hal_hw_interface` HAL component is a ROS
`hardware_interface::SystemInterface` implementation for controlling
robot joints in a real-time context.

The `controller_manager::ControllerManager` update loop runs in a HAL
thread.

## `hal_io`

The `hal_io` HAL user (non-realtime) component publishes and
subscribes to ROS `std_msgs` topics with values read from and written
to robot I/O HAL pins.  The `HAL_BIT`, `HAL_FLOAT`, `HAL_S32` and
`HAL_U32` HAL pin value types correspond to `Bool`, `Float64`, `Int32`
and `UInt32` messages from ROS `std_msgs`.

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
  - This may be installed in package form.
- The `rrbot_description` package from `gazebo_ros_demos`
  - Required by `ros_control_boilerplate` to run the
  `hal_rrbot_control` demo
  - Follow the notes in the `ros_control_boilerplate/README.md` to
    install this.

-----
## Run the demos

The `hal_rrbot_control` package contains two demos:  one for
`hal_hw_interface` and one for `hal_io`.

### `hal_hw_interface` demo

This demo runs the `ros_control_boilerplate` "RRBot" two joint
revolute-revolute robot demo with HAL.  It is meant to show how simple
it can be to build a `ros_control` hardware interface with HAL, and to
serve as an example for creating your own.

It also demonstrates the `hal_io` user component.  A simple gripper
URDF is added, and a `hal_io` service pin open the gripper when
`True` and closes when `False`.

Run the simulated hardware interface:

    ros2 launch hal_rrbot_control rrbot.launch.py
    # Debugging: append `hal_debug_output:=1 hal_debug_level:=5`

Run `halscope` to visualize HAL joint commands and feedback; in the
GUI, set the "Run Mode" to "Roll" for continuous updating:

    halscope -i hal_rrbot_control/config/hal_hw_interface.halscope

The simulated trajectories are launched directly from the
`ros2_control_demo_bringup` package:

    ros2 launch ros2_control_demo_bringup test_forward_position_controller.launch.py
    ros2 launch ros2_control_demo_bringup test_joint_trajectory_controller.launch.py

-----
## Configuration

To configure the `hal_hw_interface` and `hal_io` components, use the
sample configuration  in the `hal_rrbot_control` package as a
starting point.

### `hal_mgr` configuration

The `hal_mgr` python script is launched as a ROS node, and performs
simple management of the HAL life cycle:

- At startup,
  - Start the RTAPI real time services
  - Load a HAL configuration from a list of HAL files
- While running, spin until receiving a shutdown signal from ROS
- On shutdown, stop HAL and RTAPI and exit

The HAL configuration is specified as a list of HAL files to load from
the `halfiles` directory in the `hal_mgr/hal_files` ROS parameter.
The files may be either legacy `.hal` files or python scripts.

See the `config/hal_hw_interface.yaml` and `config/hal_io.yaml` files
in `hal_rrbot_control` for typical configuration examples.

When the hal_mgr completes loading the HAL files, it sets 'hal_mgr/ready'
topic to true. Other processes can use this topic to check the status
loading the HAL configuration.

### `hal_hw_interface` configuration

A `hal_hw_interface` configuration builds on an existing
`ros_control_boilerplate` configuration.  The
`ros_control_boilerplate` configuration file,
`rrbot_controllers.yaml`, defines joints and controllers; the
`generic_hw_control_loop` section is unneeded, since the control loop
runs in a HAL thread.

The `config/hal_hw_interface.yaml` ROS parameter file loads the
`halfiles/hal_hardware_interface.hal` file.  That file defines a new
HAL thread, loads the `hal_hw_interface` component, and adds it to the
thread.  It then sets up `limit3` components for each joint to limit
position, velocity and acceleration, and connects them to the
`hal_hw_interface` command and feedback signals.

Because the `hal_hw_interface` component is installed outside the
standard HAL component directory, its full path must be provided using
the `$COMP_DIR` environment variable:  `loadrt
$(COMP_DIR)/hal_hw_interface`.

When the `hal_hw_interface` component loads, it creates six HAL pins
for each joint:  three command output pins and three feedback input
pins for each of position, velocity and effort.  On the ROS side, it
sets up the hardware interface and controller manager.  The HAL file
adds its ros_control `update()` function to a real-time HAL thread,
and once the thread is started, the function runs in a low-latency
loop at the frequency configured for the thread.

### `hal_io` configuration

The `hal_io` user (non-real-time) component loads its configuration
from the ROS parameter server.  See the example `config/hal_io.yaml`
configuration file.  It supports the four HAL data types:  `FLOAT`,
`S32`, `U32` and `BIT`, connecting them to ROS [`std_msgs`][std_msgs]
`Float64`, `Int32`, `UInt32` and `Bool` messages, respectively.  The
configuration has three main parameters:

- `hal_io/update_rate`:  The main loop publisher update frequency in
  Hz (float)
- `hal_io/input`:  A dictionary of pin-name keys to HAL input pin type
  values; e.g. `{ "door_open" : "BIT" }` creates a HAL input pin
  `hal_io.door_open`, and publishes `Bool` messages to the topic
  `hal_io/door_open` at the `update_rate` frequency
- `hal_io/output`:  A dictionary of pin-name keys to lists of (type,
  topic) values; e.g. `{ "request_tool_num" : [ "U32",
  "robot_io/request_tool_num"] }` creates a HAL output pin
  `hal_io.request_tool_num` and sets its value from `UInt32` messages
  subscribed to from the topic `robot_io/request_tool_num`

[std_msgs]: http://wiki.ros.org/std_msgs

# TODO

- Expose joint limits through pins


# FIXME  ROS2 build

```
sudo apt-get install machinekit-hal-dev
# VERBOSE=1 \
colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --cmake-args -DCMAKE_VERBOSE_MAKEFILE=TRUE \
    --event-handlers console_cohesion+ \
    --packages-up-to hal_hw_interface
```

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
[discussion][ros_control-130]

[tormach]:  https://www.tormach.com/
[python-hal-seed]: https://github.com/machinekoder/python-hal-seed
[rtt_ros_control_example]: https://github.com/skohlbr/rtt_ros_control_example
[ros_control-130]: https://github.com/ros-controls/ros_control/issues/130
