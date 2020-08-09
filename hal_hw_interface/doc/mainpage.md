# hal_hw_interface:  Machinekit HAL ros_control component C++ API

The `hal_hw_interface` component:
* Plays out coordinated joint trajectories
* Interfaces to standard [`ros_control`][ros_control] trajectory,
  position, joint state, etc.  controllers
* Exchanges command and feedback values to pins in a [Machinekit HAL][mk-hal]
  robot configuration
* Executes in a HAL real-time thread with multi-kHz update rate and
  sub-10uS jitter (subject to CPU hardware)
* Requires a GNU/Linux computer with real-time kernel with I/O drivers
  (e.g. EtherCAT, GPIO, Beaglebone PRU, Zynq SOC)

See the [main documentation][doc-home] for more information about
using it for robot hardware integrations.

[mk-hal]: http://www.machinekit.io/docs/index-HAL/
[ros_control]: http://wiki.ros.org/ros_control
[doc-home]:  ../index.html
