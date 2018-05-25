# HAL ROS Control

A ROS `hardware_interface::RobotHW` implementation for running robot
hardware with [Machinekit][machinekit] HAL in a real-time thread.

This ROS package builds on Dave Coleman's
[`ros_control_boilerplate`][ros_control_boilerplate] to add the
following capabilities:

- Brings real-time control to ROS with Machinekit RTAPI
- Runs on inexpensive PC and ARM hardware
- Many robot hardware interfaces possible with configuration only, no
  programming
- Use Machinekit's wide variety of hardware drivers:  EtherCAT,
  GPIO step/direction, Mesa AnythingI/O FPGA, and more

The hardware interface is a subclass of Dave Coleman's
`ros_control_boilerplate` hardware interface.  The
`ros_control_boilerplate` control loop was too different to make sense
to subclass, so it was rewritten from scratch, somewhat following the
[`rtt_ros_control_example`][rtt_ros_control_example].  The C++ HAL
integration was done following examples by Bas de Bruijn and Mick
Grant.  The `hal_mgr` ROS node, which starts up the RTAPI and HAL
apparatus, was inspired by Alexander Roessler's
[`python-hal-seed`][python-hal-seed].

Coming soon:  A HAL-ROS I/O interface.

[machinekit]:  http://machinekit.io
[ros_control_boilerplate]: https://github.com/davetcoleman/ros_control_boilerplate
[rtt_ros_control_example]: https://github.com/skohlbr/rtt_ros_control_example
[python-hal-seed]: https://github.com/machinekoder/python-hal-seed

## Dependencies

The `hal_hw_interface` package has two main dependencies:

- [Machinekit][machinekit]:  Follow the directions to install from
  packages or source on a PC.
- A real-time kernel is required for low-latency control; see the
  `linux-image-rt-*` packages available on Debian.
- [`ros_control_boilerplate`][ros_control_boilerplate]:  This may be
  installed in package form.

The `hal_rrbot_control` demo has two additional dependencies:
- The demo depends on the `rrbot_control` package.  Check out the
  source of `ros_control_boilerplate` into the catkin workspace, since
  `rrbot_control` is not not included in the packaging.
- Similarly, follow the notes in the
  `ros_control_boilerplate/README.md` for installing the
  `rrbot_description` package sources in the workspace.

## Run Simulation Demo

The `hal_rrbot_control` package runs the `ros_control_boilerplate`
"RRBot" two joint revolute-revolute robot demo with HAL.  It is meant
to show how simple it can be to build a `ros_control` hardware
interface with HAL, and to serve as an example for creating your own.

Run the simulated hardware interface:

    roslaunch hal_rrbot_control hal_rrbot_simulation.launch
    # Debugging: append `hal_debug_output:=1 hal_debug_level:=5`

Run `halscope` to visualize HAL joint commands and feedback; in the
GUI, set the "Run Mode" to "Roll" for continuous updating:

    halscope -i hal_rrbot_control/config/joints.halscope

The rviz and simulated trajectories are launched identically to the
`rrbot_control` package:

    roslaunch ros_control_boilerplate rrbot_visualize.launch
    roslaunch ros_control_boilerplate rrbot_test_trajectory.launch

## TODO

- Shutdown is done poorly
  - The comp creates two threads in `rtapi_app`; these are probably
    not shut down correctly.  (Could these be registered as userspace
    threads?)
  - The exit handler might need looking at:  currently uses the
    `rtapi_app` handler; should it be updated to help with this comp?
  - Normal shutdown is poor; worse is incomplete start-up
    - E.g. try a bogus config file
- HAL I/O comp
  - I was originally thinking to combine I/O into this comp, but now
    I see reasons to keep it separate, since the boilerplate subclass
    is so small and tidy
  - The control loop should be reused; it'll need to become a template
    in that case
  - If it's kept as a ros_control hardware interface, I'll need to
    finish the `simple_controllers` and `simple_hardware_interfaces`
    packages
    - If several HAL types are implemented, these should also become
      templates

