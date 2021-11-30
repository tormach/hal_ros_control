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

#include <hal_hw_interface/hal_system_interface.hpp>
#include <hal_hw_interface/hal_handle.hpp>
#include <hal_hw_interface/hal_ros_logging.hpp>

namespace hal_system_interface
{
hardware_interface::return_type
HalSystemInterface::configure(const hardware_interface::HardwareInfo& info)
{
  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }

  HAL_ROS_INFO_NAMED(CNAME, "Initializing HAL hardware interface");

  // Initialize component
  comp_id_ = hal_init(CNAME);
  if (comp_id_ < 0)
  {
    HAL_ROS_ERR_NAMED(CNAME, "ERROR: Component creation ABORTED");
    return hardware_interface::return_type::ERROR;
  }

  HAL_ROS_INFO_NAMED(CNAME, "Initialized HAL component");

  // Initialize joints
  for (const auto& joint : info_.joints)
  {
    // Initialize joint command interfaces
    for (const auto& interface : joint.command_interfaces)
    {
      command_interfaces_.emplace_back(
          hal_hardware_interface::HalCommandInterface(
              joint.name, interface.name, interface.data_type, comp_id_));
    }

    // Initialize joint state interfaces
    for (const auto& interface : joint.state_interfaces)
    {
      state_interfaces_.emplace_back(hal_hardware_interface::HalStateInterface(
          joint.name, interface.name, interface.data_type, comp_id_));
    }
  }

  // Initialize sensors
  for (const auto& sensor : info_.sensors)
  {
    for (const auto& interface : sensor.state_interfaces)
    {
      state_interfaces_.emplace_back(hal_hardware_interface::HalStateInterface(
          sensor.name, interface.name, interface.data_type, comp_id_));
    }
  }

  HAL_ROS_INFO_NAMED(CNAME, "Initialized HAL pins");

  return hardware_interface::return_type::OK;
}  // configure()

}  // namespace hal_system_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hal_system_interface::HalSystemInterface,
                       hardware_interface::SystemInterface)
