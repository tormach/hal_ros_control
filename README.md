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
extending [`ros_control_boilerplate`][ros_control_boilerplate].  The
`hal_io` user component (non-real-time) enables simple robot I/O by
connecting HAL input and output pins with `std_msgs` publishers and
subscribers, respectively.

[machinekit]:  http://machinekit.io
[ros_control_boilerplate]: https://github.com/davetcoleman/ros_control_boilerplate

## The `hal_hw_interface` real-time component

The `hal_hw_interface` HAL component is a ROS
`hardware_interface::RobotHW` implementation for controlling robot
joints in a real-time context.

The hardware interface is a subclass of Dave Coleman's
`ros_control_boilerplate` hardware interface.  The C++ HAL integration
was done following examples by Bas de Bruijn and Mick Grant of the
Machinekit project.  The control loop runs in a HAL thread, and its
design is inspired by the
[`rtt_ros_control_example`][rtt_ros_control_example] and related
[discussion][ros_control-130].  The `hal_mgr` ROS node, which starts
up the RTAPI and HAL apparatus, is based on Alexander Roessler's
[`python-hal-seed`][python-hal-seed].

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

## Run the demos

The `hal_rrbot_control` package contains two demos:  one for
`hal_hw_interface` and one for `hal_io`.

### `hal_hw_interface` demo

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
