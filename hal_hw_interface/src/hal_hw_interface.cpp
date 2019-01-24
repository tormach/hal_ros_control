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

#include <hal_hw_interface/hal_hw_interface.h>
#include <hal_hw_interface/hal_ros_logging.h>

namespace hal_hw_interface
{
HalHWInterface::HalHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
}

void HalHWInterface::init(void (*funct)(void*, long))
{
  HAL_ROS_LOG_INFO(CNAME, "%s: Initializing HAL hardware interface", CNAME);

  // Call boilerplate init() function
  ros_control_boilerplate::GenericHWInterface::init();

  HAL_ROS_LOG_INFO(CNAME, "%s: Initialized boilerplate", CNAME);

  // Initialize component
  comp_id_ = hal_init(CNAME);
  if (comp_id_ < 0)
  {
    HAL_ROS_LOG_ERR(CNAME, "%s:  ERROR: Component creation ABORTED", CNAME);
    // return false; // FIXME
    return;
  }

  HAL_ROS_LOG_INFO(CNAME, "%s: Initialized HAL component", CNAME);

  // Initialize HAL pins for each joint
  for (std::size_t ix = 0; ix < num_joints_; ix++)
  {
    // init_joint(ix);
    HAL_ROS_LOG_INFO(CNAME, "%s: Init joint #%zu %s", CNAME, ix,
                     joint_names_[ix].c_str());

    if (!create_joint_float_pins(ix, &joint_pos_cmd_ptrs_, HAL_OUT, "pos-"
                                                                    "cmd") ||
        !create_joint_float_pins(ix, &joint_vel_cmd_ptrs_, HAL_OUT, "vel-"
                                                                    "cmd") ||
        !create_joint_float_pins(ix, &joint_eff_cmd_ptrs_, HAL_OUT, "eff-"
                                                                    "cmd") ||
        !create_joint_float_pins(ix, &joint_pos_fb_ptrs_, HAL_IN, "pos-fb") ||
        !create_joint_float_pins(ix, &joint_vel_fb_ptrs_, HAL_IN, "vel-fb") ||
        !create_joint_float_pins(ix, &joint_eff_fb_ptrs_, HAL_IN, "eff-fb"))
    {
      HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint %zu %s.%s", CNAME,
                      ix, CNAME, joint_names_[ix].c_str());
      // return false; // FIXME
      return;
    }
  }

  // Initialize started pin
  if (!create_bit_pin(&reset_ptr_, HAL_IN, "reset"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize reset pin", CNAME);
    // return false; // FIXME
    return;
  }

  HAL_ROS_LOG_INFO(CNAME, "%s:  Initialized HAL pins", CNAME);

  // Export the function
  if (hal_export_functf(funct, this, 1, 0, comp_id_, "%s.funct", CNAME) < 0)
  {
    HAL_ROS_LOG_INFO(CNAME, "%s: ERROR: hal_export_functf failed", CNAME);
    hal_exit(comp_id_);
    // return false; // FIXME
    return;
  }
  HAL_ROS_LOG_INFO(CNAME, "%s:  Exported HAL function", CNAME);

  // Mark component ready
  hal_ready(comp_id_);

  HAL_ROS_LOG_INFO(CNAME, "%s:  HAL component ready!", CNAME);

  // return true; // FIXME
}  // init()

bool HalHWInterface::create_joint_float_pins(const std::size_t ix,
                                             std::vector<double**>* ptrs,
                                             hal_pin_dir_t dir,
                                             const char* name)
{
  // Sanity check vector length
  if (ptrs->size() != ix)
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Size of pin storage not consistent with ID",
                    CNAME);
    return false;
  }
  // Allocate space
  ptrs->push_back((hal_float_t**)hal_malloc(sizeof(hal_float_t*)));
  if (ptrs->at(ix) == NULL)
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Allocate HAL pin failed", CNAME);
    return false;
  }
  if (hal_pin_float_newf(dir, ptrs->at(ix), comp_id_, "%s.%s.%s", CNAME,
                         joint_names_[ix].c_str(), name))
  {
    HAL_ROS_LOG_INFO(CNAME, "%s: New HAL pin %s.%s.%s failed", CNAME, CNAME,
                     joint_names_[ix].c_str(), name);
    return false;
  }

  HAL_ROS_LOG_INFO(CNAME, "%s: New HAL pin %s.%s.%s succeeded; addr %p", CNAME,
                   CNAME, joint_names_[ix].c_str(), name, ptrs->at(ix));
  return true;
}

bool HalHWInterface::create_bit_pin(bool*** ptr, hal_pin_dir_t dir,
                                    const char* name)
{
  // Allocate space
  *ptr = ((hal_bit_t**)hal_malloc(sizeof(hal_bit_t*)));
  if (*ptr == NULL)
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Allocate HAL pin failed", CNAME);
    return false;
  }
  if (hal_pin_bit_newf(dir, *ptr, comp_id_, "%s.%s", CNAME, name))
  {
    HAL_ROS_LOG_INFO(CNAME, "%s: New HAL pin %s.%s failed", CNAME, CNAME, name);
    return false;
  }

  HAL_ROS_LOG_INFO(CNAME, "%s: New HAL pin %s.%s succeeded; addr %p", CNAME,
                   CNAME, name, *ptr);
  return true;
}

void HalHWInterface::read(ros::Duration& elapsed_time)
{
  // Copy HAL joint feedback pin values to controller joint states
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    joint_position_[joint_id] = **joint_pos_fb_ptrs_[joint_id];
    joint_velocity_[joint_id] = **joint_vel_fb_ptrs_[joint_id];
    joint_effort_[joint_id] = **joint_eff_fb_ptrs_[joint_id];
  }

  // Read reset pin
  reset_controllers = **reset_ptr_;
}

void HalHWInterface::write(ros::Duration& elapsed_time)
{
  // Enforce joint limits
  enforceLimits(elapsed_time);
  // Copy controller joint command values to HAL joint command pins
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    **joint_pos_cmd_ptrs_[joint_id] = joint_position_command_[joint_id];
    **joint_vel_cmd_ptrs_[joint_id] = joint_velocity_command_[joint_id];
    **joint_eff_cmd_ptrs_[joint_id] = joint_effort_command_[joint_id];
  }
}

void HalHWInterface::enforceLimits(ros::Duration& period)
{
  // FIXME how does this fit in?  Should it just be done in HAL?
  // from sim_hw_interface.cpp:
  // pos_jnt_sat_interface_.enforceLimits(period);
}

void HalHWInterface::shutdown()
{
  if (!comp_id_)
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: HAL already shut down", CNAME);
  }
  else
  {
    HAL_ROS_LOG_INFO(CNAME, "%s: HAL shutting down", CNAME);
    hal_exit(comp_id_);
    comp_id_ = 0;
  }
}

}  // namespace
