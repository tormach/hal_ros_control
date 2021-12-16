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

#ifndef HAL_HW_INTERFACE__HAL_HANDLE_HPP_
#define HAL_HW_INTERFACE__HAL_HANDLE_HPP_

#include <string>
#include <unordered_map>

#include "hardware_interface/handle.hpp"
#include "hal_hw_interface/hal_def.hpp"
#include "hal_hw_interface/hal_ros_logging.hpp"
namespace hal_hardware_interface
{
class HalReadOnlyHandle : public hardware_interface::ReadOnlyHandle
{
public:
  HalReadOnlyHandle(const std::string& name, const std::string& interface_name,
                    const std::string& data_type, int comp_id)
    : hardware_interface::ReadOnlyHandle(name, interface_name)
    , comp_id_(comp_id)
  {
    alloc_and_init_hal_pin(data_type);
  }

  explicit HalReadOnlyHandle(const std::string& interface_name,
                             const std::string data_type, int comp_id)
    : hardware_interface::ReadOnlyHandle(interface_name), comp_id_(comp_id)
  {
    alloc_and_init_hal_pin(data_type);
  }

  explicit HalReadOnlyHandle(const char* interface_name, std::string data_type,
                             int comp_id)
    : hardware_interface::ReadOnlyHandle(interface_name), comp_id_(comp_id)
  {
    alloc_and_init_hal_pin(data_type);
  }

  /// Returns true if handle references a value.
  inline operator bool() const
  {
    return value_ptr_ptr_ != nullptr;
  }

  double get_value() const
  {
    THROW_ON_NULLPTR(value_ptr_ptr_);
    THROW_ON_NULLPTR(*value_ptr_ptr_);
    return **value_ptr_ptr_;
  }

  const std::string pin_name() const
  {
    return name_.empty() ? interface_name_ : (name_ + '.' + interface_name_);
  }

  void alloc_and_init_hal_pin(std::string data_type)
  {
    const std::unordered_map<std::string, int> type_string_case_map{
      { "double", 1 }, { "float", 1 }, { "bit", 2 },  { "bool", 2 },
      { "int", 3 },    { "s32", 3 },   { "uint", 4 }, { "u32", 4 },
    };

    switch (type_string_case_map.at(data_type))
    {
      case 1:  // double/float
        alloc_and_init_hal_pin_typed<hal_float_t>();
        break;
      case 2:  // bit/bool
        alloc_and_init_hal_pin_typed<hal_bit_t>();
        break;
      case 3:  // int/s32
        alloc_and_init_hal_pin_typed<hal_s32_t>();
        break;
      case 4:  // uint/u32
        alloc_and_init_hal_pin_typed<hal_u32_t>();
        break;
    }
  }

protected:
  template <class hal_t>
  void alloc_and_init_hal_pin_typed()
  {
    hal_t** ptr = (reinterpret_cast<hal_t**>(hal_malloc(sizeof(hal_t*))));
    if (ptr == nullptr)
    {
      HAL_ROS_ERR_NAMED(CNAME, "Failed to allocate HAL pin %s",
                        pin_name().c_str());
      throw std::runtime_error(std::string("Failed to init handle '") +
                               get_full_name() + "'");
    }
    if (hal_pin_newf(&ptr))
    {
      HAL_ROS_ERR_NAMED(CNAME, "New HAL pin %s failed", pin_name().c_str());
      throw std::runtime_error(std::string("Failed to init handle '") +
                               get_full_name() + "'");
    }
    HAL_ROS_INFO_NAMED(CNAME, "New HAL pin %s succeeded", pin_name().c_str());
  }

  // Polymorphic member functions to create HAL pins of appropriate type
  int hal_pin_newf(hal_bit_t*** ptr)
  {
    return hal_pin_bit_newf(pin_dir_, *ptr, comp_id_, "%s", pin_name().c_str());
  }

  int hal_pin_newf(hal_float_t*** ptr)
  {
    return hal_pin_float_newf(pin_dir_, *ptr, comp_id_, "%s",
                              pin_name().c_str());
  }

  int hal_pin_newf(hal_u32_t*** ptr)
  {
    return hal_pin_u32_newf(pin_dir_, *ptr, comp_id_, "%s", pin_name().c_str());
  }

  int hal_pin_newf(hal_s32_t*** ptr)
  {
    return hal_pin_s32_newf(pin_dir_, *ptr, comp_id_, "%s", pin_name().c_str());
  }

  const hal_pin_dir_t pin_dir_ = HAL_IN;
  double** value_ptr_ptr_;
  int comp_id_;
};

class HalReadWriteHandle : public HalReadOnlyHandle,
                           public hardware_interface::ReadWriteHandle
{
public:
  HalReadWriteHandle(const std::string& name, const std::string& interface_name,
                     const std::string& data_type, int comp_id)
    : HalReadOnlyHandle(name, interface_name, data_type, comp_id)
    , hardware_interface::ReadWriteHandle(name, interface_name)
  {
  }

  explicit HalReadWriteHandle(const std::string& interface_name,
                              const std::string& data_type, int comp_id)
    : HalReadOnlyHandle(interface_name, data_type, comp_id)
    , hardware_interface::ReadWriteHandle(interface_name)
  {
  }

  explicit HalReadWriteHandle(const char* interface_name,
                              const std::string& data_type, int comp_id)
    : HalReadOnlyHandle(interface_name, data_type, comp_id)
    , hardware_interface::ReadWriteHandle(interface_name)
  {
  }

  void set_value(double value)
  {
    THROW_ON_NULLPTR(this->value_ptr_ptr_);
    THROW_ON_NULLPTR(*this->value_ptr_ptr_);
    **this->value_ptr_ptr_ = value;
  }

protected:
  const hal_pin_dir_t pin_dir_ = HAL_OUT;
};

class HalStateInterface : public HalReadOnlyHandle,
                          public hardware_interface::StateInterface
{
public:
  HalStateInterface(const std::string& name, const std::string& interface_name,
                    const std::string& data_type, int comp_id)
    : HalReadOnlyHandle(name, interface_name, data_type, comp_id)
    , hardware_interface::StateInterface(name, interface_name)
  {
  }
};

class HalCommandInterface : public HalReadWriteHandle,
                            public hardware_interface::CommandInterface
{
public:
  HalCommandInterface(const std::string& name,
                      const std::string& interface_name,
                      const std::string& data_type, int comp_id)
    : HalReadWriteHandle(name, interface_name, data_type, comp_id)
    , hardware_interface::CommandInterface(name, interface_name)
  {
  }
};

}  // namespace hal_hardware_interface

#endif  // HAL_HW_INTERFACE__HAL_HANDLE_HPP_
