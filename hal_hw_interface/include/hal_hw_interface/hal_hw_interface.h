// Copyright (c) 2018, John Morris
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//     * Redistributions of source code must retain the above
//       copyright notice, this list of conditions and the following
//       disclaimer.
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials
//       provided with the distribution.
//     * Neither the name of the <organization> nor the names of its
//       contributors may be used to endorse or promote products
//       derived from this software without specific prior written
//       permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
// <COPYRIGHT HOLDER> BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
// USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
// SUCH DAMAGE.

#ifndef HAL_HW_INTERFACE_HAL_HW_INTERFACE_H
#define HAL_HW_INTERFACE_HAL_HW_INTERFACE_H

// hal_hw_interface subclasses ros_control_boilerplate
#include <ros_control_boilerplate/generic_hw_interface.h>
#include <machinekit_interfaces/realtime_event_interface.h>
#include <machinekit_interfaces/probe_interface.h>
#include <machinekit_interfaces/joint_event_interface.h>

// HAL
#include <hal.h>

// ROS
#include <ros/duration.h>

// Component name
#define CNAME "hal_hw_interface"

namespace hal_hw_interface
{
/**
* \brief A `ros_control_boilerplate::GenericHWInterface` subclass for Machinekit
* HAL
*
* The `hal_hw_interface::HalHWInterface` class implements the Machinekit HAL
* realtime component:
* 1. Initializes the component
* 2. Implements the ros_control `read()` and `write()` functions
* 3. Shuts down the component
*
* The HAL component name is `hw_hw_interface`, and has one `reset` pin and six
* pins for each joint.
*
* The `reset` pin resets the ROS controllers whenever it is high.
*
* Joint names are read from configuration in [`ros_control_boilerplate`][1].
* Six HAL pins are created for each joint:
*
* * Output pins connecting joint command from ROS into HAL
*   * `<joint>.pos-cmd`, `<joint>.vel-cmd` and `<joint>.eff-cmd`
* * Input pins connecting joint feedback from HAL back to ROS
*   * `<joint>.pos-fb`, `<joint>.vel-fb` and `<joint>.eff-fb`
*
* The `read()` function reads joint feedback values from the `<joint>.*-fb` HAL
* pins into the `hardware_interface::JointHandle`, and the `write()` function
* writes joint command values back out to the `<joint>.*-cmd` HAL pins.
*
* This is plumbed into a ROS node in the `hal_hw_interface::HalRosControlLoop`
* class.
*
* [1]: https://github.com/PickNikRobotics/ros_control_boilerplate
*/

class HalHWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh          ROS node handle
   * \param urdf_model  Optional pointer to a parsed robot model
   */
  HalHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /**
   * \brief Initialize the hardware interface
   * \param funct  The component function run periodically in a HAL RT thread
   *
   * Initializes the HAL component and sets up HAL pins for each joint.
   */
  //* \todo Give this an int return value for reporting failure
  //* \todo Make the `reset` pin an IO pin
  void init_hal(void (*funct)(void*, long));

  /**
   * \brief Create float-type HAL pins for each joint
   * \param ix    Pin
   * \param ptrs  A vector of double pointers to HAL float storage
   * \param dir   Pin direction; one of HAL_IN, HAL_OUT
   * \param name  A string suffix to append to the pin name
   *
   * Used in `init()`
   */
  bool create_joint_float_pins(const std::size_t ix,
                               std::vector<double**>* ptrs, hal_pin_dir_t dir,
                               const char* name);

  /**
   * \brief Create single bit-type HAL pin
   * \param ptr   A bool pointer to HAL hal_bit_t storage
   * \param dir   Pin direction; one of HAL_IN, HAL_OUT
   * \param name  A string suffix to append to the pin name
   *
   * Used in `init()`
   */
  bool create_bit_pin(bool*** ptr, hal_pin_dir_t dir, const char* name);

  bool create_s32_pin(int*** ptr, hal_pin_dir_t dir, const char* name);

  /**
   * \brief Read the state from the robot hardware.
   * \param elapsed_time - period since last run
   */
  void read(ros::Duration& elapsed_time);

  /**
   * \brief Tell control loop whether controller reset is needed in update()
   */
  bool reset_controllers;

  /**
   * \brief Write the command to the robot hardware.
   * \param elapsed_time  Period since last run
   */
  void write(ros::Duration& elapsed_time);

  /**
   * \brief Enforce joint limits
   * \param period   Period since last run
   */
  //! \todo Unimplemented
  void enforceLimits(ros::Duration& period);

  /**
   * \brief Shut down the HAL component and the ROS node
   */
  void shutdown();

protected:
  /**
   * \brief HAL component ID
   */
  int comp_id_;

  /** indicates if the probe signal is active in HAL */
  int probe_signal;
  int probe_transition;

  /** Are we expecting a probe trip? */
  int probe_request_capture_type;

  std::vector<double> probe_joint_position_;
  std::vector<double> probe_joint_velocity_;
  std::vector<double> probe_joint_effort_;

  machinekit_interfaces::ProbeInterface probe_interface;
  machinekit_interfaces::JointEventDataInterface joint_event_data_interface_;

private:
  // Joints:  HAL storage
  // - Commands
  //!     Joint position command value pointer vector
  std::vector<double**> joint_pos_cmd_ptrs_;
  //!     Joint velocity command value pointer vector
  std::vector<double**> joint_vel_cmd_ptrs_;
  //!     Joint effort command value pointer vector
  std::vector<double**> joint_eff_cmd_ptrs_;
  // - States
  //!     Joint position feedback value pointer vector
  std::vector<double**> joint_pos_fb_ptrs_;
  //!     Joint velocity feedback value pointer vector
  std::vector<double**> joint_vel_fb_ptrs_;
  //!     Joint effort feedback value pointer vector
  std::vector<double**> joint_eff_fb_ptrs_;

  //!     Probe Position result
  std::vector<double**> probe_joint_result_ptrs_; // HAL output pins for probe result (reference)

  // TODO hal pin for realtime safety input

  // TODO condense these with the preview member variables to remove the indirection / extra copy
  bool** reset_ptr_;  // HAL input pin for controller reset
  bool** probe_signal_ptr_;  // HAL input pin, probe signal
  int** probe_transition_ptr_;  // HAL output pin for detected probe transition (reference)
  int** probe_capture_ptr_;  // HAL output pin for expected capture type (reference)

};  // HalHWInterface

}  // hardware_interface

#endif  // HAL_HW_INTERFACE_HAL_HW_INTERFACE_H
