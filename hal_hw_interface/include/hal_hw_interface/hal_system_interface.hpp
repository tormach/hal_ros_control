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

#ifndef HAL_HW_INTERFACE__HAL_SYSTEM_INTERFACE_HPP_
#define HAL_HW_INTERFACE__HAL_SYSTEM_INTERFACE_HPP_

// HAL
#define RTAPI 1
#include <hal.h>

#include <vector>
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/macros.hpp"
#include "hal_hw_interface/hal_def.hpp"
#include "hal_hw_interface/visibility_control.h"
#include "hal_hw_interface/hal_handle.hpp"

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace hal_system_interface
{
/**
 * \brief A `ros_control_boilerplate::GenericHWInterface` subclass for
 * Machinekit HAL
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

class HalSystemInterface : public hardware_interface::BaseInterface<
                               hardware_interface::SystemInterface>
{
public:
  // Define aliases and static functions for using the Class with shared_ptrs
  RCLCPP_SHARED_PTR_DEFINITIONS(HalSystemInterface);

  HAL_HW_INTERFACE_PUBLIC
  hardware_interface::return_type
  configure(const hardware_interface::HardwareInfo& info) override;

  HAL_HW_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  HAL_HW_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  // HAL_HW_INTERFACE_PUBLIC
  // hardware_interface::return_type prepare_command_mode_switch(
  //   const std::vector<std::string> & /*start_interfaces*/,
  //   const std::vector<std::string> & /*stop_interfaces*/) override;

  // HAL_HW_INTERFACE_PUBLIC
  // hardware_interface::return_type perform_command_mode_switch(
  //   const std::vector<std::string> & /*start_interfaces*/,
  //   const std::vector<std::string> & /*stop_interfaces*/) override;

  HAL_HW_INTERFACE_PUBLIC
  hardware_interface::return_type start() override;

  HAL_HW_INTERFACE_PUBLIC
  hardware_interface::return_type stop() override;

  HAL_HW_INTERFACE_PUBLIC
  hardware_interface::return_type read() override;

  HAL_HW_INTERFACE_PUBLIC
  hardware_interface::return_type write() override;

protected:
  /**
   * \brief HAL component ID
   */
  int comp_id_;

  // Interfaces
  std::vector<hal_hardware_interface::HalCommandInterface> command_interfaces_;
  std::vector<hal_hardware_interface::HalStateInterface> state_interfaces_;
};  // HalSystemInterface

}  // namespace hal_system_interface

#endif  // HAL_HW_INTERFACE__HAL_SYSTEM_INTERFACE_HPP_
