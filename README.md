# ROS HAL interface

This package provides interfaces to [Machinekit][machinekit] HAL.  The
`hal_hw_interface` HAL RT component extends
[`ros_control_boilerplate`][ros_control_boilerplate] to control robot
joints in HAL.  The `hal_io_publisher` HAL user component publishes
robot I/O pins from HAL to ROS using `std_msgs`.

[machinekit]:  http://machinekit.io
[ros_control_boilerplate]: https://github.com/davetcoleman/ros_control_boilerplate

## `hal_hw_interface`

A ROS `hardware_interface::RobotHW` implementation for running robot
hardware with Machinekit HAL in a real-time thread.

This ROS package adds the following capabilities to Dave Coleman's
`ros_control_boilerplate`:

- Brings real-time control to ROS with Machinekit RTAPI
- Runs on inexpensive PC and ARM hardware
- Many robot hardware interfaces possible with configuration only, no
  programming
- Use Machinekit's wide variety of hardware drivers:  EtherCAT,
  GPIO step/direction, Mesa AnythingI/O FPGA, and more

The hardware interface is a subclass of the `ros_control_boilerplate`
hardware interface.  The `ros_control_boilerplate` control loop was
too different to make sense to subclass, so it was rewritten from
scratch, somewhat following the
[`rtt_ros_control_example`][rtt_ros_control_example] and related
[discussion][ros_control-130].  The C++ HAL integration was done
following examples by Bas de Bruijn and Mick Grant.  The `hal_mgr` ROS
node, which starts up the RTAPI and HAL apparatus, was inspired by
Alexander Roessler's [`python-hal-seed`][python-hal-seed].

[rtt_ros_control_example]: https://github.com/skohlbr/rtt_ros_control_example
[ros_control-130]: https://github.com/ros-controls/ros_control/issues/130
[python-hal-seed]: https://github.com/machinekoder/python-hal-seed

## `hal_io`

The `hal_io` HAL user (non-realtime) component publishes and
subscribes to ROS `std_msgs` topics with values read from and written
to robot I/O HAL pins.  The `HAL_BIT`, `HAL_FLOAT`, `HAL_S32` and
`HAL_U32` HAL pin value types correspond to `Bool`, `Float64`, `Int32`
and `UInt32` messages from ROS `std_msgs`.

## Dependencies

The `hal_io_publisher` has two main dependencies:
- [Machinekit][machinekit]:  Follow the directions to install from
  packages or source on a PC.
- A real-time kernel is required for low-latency control; see the
  `linux-image-rt-*` packages available on Debian.

The `hal_hw_interface` package adds an additional dependency:
- [`ros_control_boilerplate`][ros_control_boilerplate]:  This may be
  installed in package form.

The `hal_rrbot_control` demo has two additional dependencies:
- The demo depends on the `rrbot_control` package.  Check out the
  source of `ros_control_boilerplate` into the catkin workspace, since
  `rrbot_control` is not not included in the packaging.
- Similarly, follow the notes in the
  `ros_control_boilerplate/README.md` for installing the
  `rrbot_description` package sources in the workspace.

## Run the Demos

The `hal_rrbot_control` package contains two demos:  one for
`hal_hw_interface` and one for `hal_io`.

### `hal_hw_interface` Demo

This demo runs the `ros_control_boilerplate` "RRBot" two joint
revolute-revolute robot demo with HAL.  It is meant to show how simple
it can be to build a `ros_control` hardware interface with HAL, and to
serve as an example for creating your own.

Run the simulated hardware interface:

    roslaunch hal_rrbot_control hal_rrbot_simulation.launch
    # Debugging: append `hal_debug_output:=1 hal_debug_level:=5`

Run `halscope` to visualize HAL joint commands and feedback; in the
GUI, set the "Run Mode" to "Roll" for continuous updating:

    halscope -i hal_rrbot_control/config/hal_hw_interface.halscope

The rviz and simulated trajectories are launched identically to the
`rrbot_control` package:

    roslaunch hal_rrbot_control rrbot_visualize.launch
    roslaunch hal_rrbot_control rrbot_test_trajectory.launch

### `hal_io` demo

The `hal_io_demo.launch` file loads the `hal_io.yaml` configuration.
For each of the four supported data types, it creates one HAL input
pin with ROS publisher connected to one ROS subscriber with HAL output
pin, thus creating a HAL to ROS to HAL feedback loop.  The
`hal_io.hal` file then generates a sine wave, feeding it to the HAL
input pins (converting as appropriate).  A `halscope` will show the
output pins tracking the input pins.

This demo is a contrived example, but it shows working connections
from HAL to ROS and back, and gives an example of how to configure the
HAL pins and ROS publishers and subscribers.  A real application would
connect its own publishers and subscribers to those from `hal_io`.

Run the I/O demo:

    roslaunch hal_rrbot_control hal_io_demo.launch
    # Debugging: append `hal_debug_output:=1 hal_debug_level:=5`

Run `halscope` to visualize the HAL pins; in the GUI, set the "Run
Mode" to "Roll" for continuous updating:

    halscope -i hal_rrbot_control/config/hal_io.halscope

Run `rqt_plot` to visualize the ROS messages:

    rqt_plot /rrbot/hal_io/bool_out:float_out:int_out:uint_out

## TODO

- Shutdown is done poorly
  - The comp creates two threads in `rtapi_app`; these are probably
    not shut down correctly.  (Could these be registered as userspace
    threads?)
  - The exit handler might need looking at:  currently uses the
    `rtapi_app` handler; should it be updated to help with this comp?
  - Normal shutdown is poor; worse is incomplete start-up
    - E.g. try a bogus config file
