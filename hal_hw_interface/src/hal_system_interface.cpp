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

// Disable warnings that need to be fixed in external headers
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#include <hal/hal.h>       // HAL public API decls
#include <hal/hal_priv.h>  // halpr_find_comp_by_name
#pragma GCC diagnostic pop

#include <hal_hw_interface/hal_system_interface.hpp>
#include <hal_hw_interface/hal_ros_logging.hpp>

#include <string>
#include <vector>

namespace hal_system_interface
{
static inline std::string joint_intf_name(std::string joint_name,
                                          std::string intf_name,
                                          std::string suffix = "")
{
  return joint_name + "/" + intf_name + suffix;
}

hal_float_t** HalSystemInterface::alloc_and_init_hal_pin(
    const std::string base_name, const std::string interface_name,
    const std::string suffix, const hal_pin_dir_t pin_dir)
{
  std::string pin_name =
      info_.name + '.' + base_name + '.' + interface_name + suffix;

  hal_float_t** ptr =
      (reinterpret_cast<hal_float_t**>(hal_malloc(sizeof(hal_float_t*))));
  if (ptr == nullptr)
  {
    HAL_ROS_ERR_NAMED(LOG_NAME, "Failed to allocate HAL pin %s",
                      pin_name.c_str());
    throw std::runtime_error(std::string("Failed to init HAL pin '") +
                             pin_name + "'");
  }
  if (hal_pin_float_newf(pin_dir, ptr, comp_id_, "%s", pin_name.c_str()))
  {
    HAL_ROS_ERR_NAMED(LOG_NAME, "New HAL pin %s failed", pin_name.c_str());
    throw std::runtime_error("Failed to init HAL pin '" + pin_name + "'");
  }

  return ptr;
}

void HalSystemInterface::init_command_interface(
    const std::string joint_name, const std::string interface_name,
    const std::string data_type)
{
  if (data_type != "double")
    throw std::runtime_error("Interface " + joint_name + " " + interface_name +
                             " type  " + data_type + ", expected 'double'");
  auto name = joint_intf_name(joint_name, interface_name, "_cmd");
  double** hal_pin_storage =
      alloc_and_init_hal_pin(joint_name, interface_name, "_cmd", HAL_OUT);
  command_intf_data_map_[name] = { .base_name = joint_name,
                                   .interface_name = interface_name,
                                   .hal_pin_storage = hal_pin_storage,
                                   .handle_storage = 0.0 };
}

void HalSystemInterface::init_state_interface(const std::string joint_name,
                                              const std::string interface_name,
                                              const std::string data_type)
{
  if (data_type != "double")
    throw std::runtime_error("Interface " + joint_name + " " + interface_name +
                             " type  " + data_type + ", expected 'double'");
  auto name = joint_intf_name(joint_name, interface_name, "_fb");
  double** hal_pin_storage =
      alloc_and_init_hal_pin(joint_name, interface_name, "_fb", HAL_IN);
  state_intf_data_map_[name] = { .base_name = joint_name,
                                 .interface_name = interface_name,
                                 .hal_pin_storage = hal_pin_storage,
                                 .handle_storage = 0.0 };
}

hardware_interface::return_type
HalSystemInterface::configure(const hardware_interface::HardwareInfo& info)
{
  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }

  HAL_ROS_INFO_NAMED(LOG_NAME, "Initializing HAL hardware interface");

  // Get HAL comp id
  comp_id_ = halpr_find_comp_by_name(CNAME)->hdr._id;
  HAL_ROS_INFO_NAMED(LOG_NAME, "HAL component %s ID:  %d", CNAME, comp_id_);

  // Initialize joints
  for (const auto& joint : info_.joints)
  {
    // Initialize joint command interfaces
    for (const auto& interface : joint.command_interfaces)
    {
      init_command_interface(joint.name, interface.name, interface.data_type);
    }

    // Initialize joint state interfaces
    for (const auto& interface : joint.state_interfaces)
    {
      init_state_interface(joint.name, interface.name, interface.data_type);
    }
  }

  // Initialize sensors
  for (const auto& sensor : info_.sensors)
  {
    for (const auto& interface : sensor.state_interfaces)
    {
      init_state_interface(sensor.name, interface.name, interface.data_type);
    }
  }

  HAL_ROS_INFO_NAMED(LOG_NAME, "Initialized HAL pins");

  return hardware_interface::return_type::OK;
}  // configure()

std::vector<hardware_interface::StateInterface>
HalSystemInterface::export_state_interfaces()
{
  HAL_ROS_INFO_NAMED(LOG_NAME, "Exporting state interfaces");
  std::vector<hardware_interface::StateInterface> interfaces;
  for (auto& [name, intf_data] : state_intf_data_map_)
    interfaces.emplace_back(intf_data.base_name, intf_data.interface_name,
                            &intf_data.handle_storage);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
HalSystemInterface::export_command_interfaces()
{
  HAL_ROS_INFO_NAMED(LOG_NAME, "Exporting command interfaces");
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (auto& [name, intf_data] : command_intf_data_map_)
    interfaces.emplace_back(intf_data.base_name, intf_data.interface_name,
                            &intf_data.handle_storage);
  return interfaces;
}

hardware_interface::return_type HalSystemInterface::read()
{
  for (auto& [name, intf_data] : state_intf_data_map_)
    // Copy to handle from HAL pin
    intf_data.handle_storage = **intf_data.hal_pin_storage;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HalSystemInterface::write()
{
  for (auto& [name, intf_data] : command_intf_data_map_)
    // Copy to HAL pin from handle
    **intf_data.hal_pin_storage = intf_data.handle_storage;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HalSystemInterface::start()
{
  HAL_ROS_INFO_NAMED(LOG_NAME, "Starting HAL system interface");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HalSystemInterface::stop()
{
  HAL_ROS_INFO_NAMED(LOG_NAME, "Stopping HAL system interface");
  return hardware_interface::return_type::OK;
}

}  // namespace hal_system_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hal_system_interface::HalSystemInterface,
                       hardware_interface::SystemInterface)
