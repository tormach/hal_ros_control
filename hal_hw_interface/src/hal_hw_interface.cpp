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
#include <stop_event_msgs/SetNextProbeMove.h>

static constexpr const char* VER_DESCRIPTION = "Probing development version "
                                               "0.1";

namespace hal_hw_interface
{
HalHWInterface::HalHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
}

int HalHWInterface::init_hal(void (*funct)(void*, long))
{
  HAL_ROS_LOG_INFO(CNAME,
                   "%s: Initializing HAL hardware interface, description: %s",
                   CNAME, VER_DESCRIPTION);

  // Register handles for joint position at probe trip

  num_joints_ = joint_names_.size();
  probe_joint_position_.resize(num_joints_, 0.0);
  probe_joint_velocity_.resize(num_joints_, 0.0);
  probe_joint_effort_.resize(num_joints_, 0.0);
  joint_velocity_prev_.resize(num_joints_, 0.0);

  // Initialize interfaces for probe position (done deliberately before the init
  // below since that's where the interfaces are registered)
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    ROS_INFO_STREAM_NAMED(name_, "Setting up handle for probe position for "
                                     << joint_names_[joint_id]);

    // Create joint state interface
    joint_event_data_interface_.registerHandle(
        machinekit_interfaces::JointEventDataHandle(
            joint_names_[joint_id] + "_probe",
            &(probe_joint_position_[joint_id]),
            &(probe_joint_velocity_[joint_id]),
            &(probe_joint_effort_[joint_id])));
  }  // end for each joint
  registerInterface(&joint_event_data_interface_);

  hal_s32_pin_interface_.registerHandle(machinekit_interfaces::HALS32PinHandle(
      "controller_status", &error_code_));
  registerInterface(&hal_s32_pin_interface_);

  hal_bit_pin_interface_.registerHandle(
      machinekit_interfaces::HALBitPinHandle("estop", &estop_event_));
  hal_bit_pin_interface_.registerHandle(
      machinekit_interfaces::HALBitPinHandle("stop", &stop_event_));
  registerInterface(&hal_bit_pin_interface_);

  hal_bit_pin_interface_.registerHandle(
      machinekit_interfaces::HALBitPinHandle("safety_input", &safety_input_event_));
  hal_bit_pin_interface_.registerHandle(
      machinekit_interfaces::HALBitPinHandle("enabling_input", &enabling_input_event_));
  registerInterface(&hal_bit_pin_interface_);

 // TODO
  // Call base class init to set register interfaces and handles for joint state
  // / command / limits
  ros_control_boilerplate::GenericHWInterface::init();

  // TODO look up the probe name in config instead of hard-coding it
  // TODO support multiple probes
  probe_interface_.registerHandle(machinekit_interfaces::ProbeHandle(
      "probe", &probe_request_capture_type_, &probe_signal_, &probe_transition_,
      &probe_result_type_, &probe_event_time_));
  registerInterface(&probe_interface_);
  HAL_ROS_LOG_INFO(CNAME, "%s: Initialized probe / stop interfaces", CNAME);

  // Initialize PosVel interfaces for each joint
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Setting up PosVel interface for joint name: "
                                      << joint_names_[joint_id]);

    // Add command interfaces to joints
    // TODO: decide based on transmissions?
    hardware_interface::PosVelJointHandle joint_handle_posvel =
        hardware_interface::PosVelJointHandle(
            joint_state_interface_.getHandle(joint_names_[joint_id]),
            &joint_position_command_[joint_id],
            &joint_velocity_command_[joint_id]);

    pos_vel_joint_interface_.registerHandle(joint_handle_posvel);
  }  // end for each joint

  registerInterface(&pos_vel_joint_interface_);

  HAL_ROS_LOG_INFO(CNAME, "%s: Initialized PosVel handles / interface", CNAME);

  // Initialize component
  comp_id_ = hal_init(CNAME);
  if (comp_id_ < 0)
  {
    HAL_ROS_LOG_ERR(CNAME, "%s:  ERROR: Component creation ABORTED", CNAME);
    return false;
  }

  HAL_ROS_LOG_INFO(CNAME, "%s: Initialized HAL component", CNAME);

  // Initialize HAL pins for each joint
  for (std::size_t ix = 0; ix < num_joints_; ix++)
  {
    // init_joint(ix);
    HAL_ROS_LOG_INFO(CNAME, "%s: Init joint #%zu %s", CNAME, ix,
                     joint_names_[ix].c_str());

    if (!create_joint_float_pins(ix, &joint_pos_cmd_ptrs_, HAL_OUT,
                                 "pos-"
                                 "cmd") ||
        !create_joint_float_pins(ix, &joint_vel_cmd_ptrs_, HAL_OUT,
                                 "vel-"
                                 "cmd") ||
        !create_joint_float_pins(ix, &joint_eff_cmd_ptrs_, HAL_OUT,
                                 "eff-"
                                 "cmd") ||
        !create_joint_float_pins(ix, &probe_joint_result_ptrs_, HAL_OUT,
                                 "probe-pos") ||
        !create_joint_float_pins(ix, &joint_ferror_ptrs_, HAL_OUT, "ferror") ||
        !create_joint_float_pins(ix, &joint_pos_fb_ptrs_, HAL_IN, "pos-fb") ||
        !create_joint_float_pins(ix, &joint_vel_fb_ptrs_, HAL_IN, "vel-fb") ||
        !create_joint_float_pins(ix, &joint_eff_fb_ptrs_, HAL_IN, "eff-fb") ||
        !create_joint_float_pins(ix, &joint_accel_ptrs_, HAL_OUT, "accel"))
    {
      HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint %zu %s.%s", CNAME,
                      ix, CNAME, joint_names_[ix].c_str());
      return false;
    }
  }

  // Initialize reset pin
  if (!create_bit_pin(&reset_ptr_, HAL_IN, "reset"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize reset pin", CNAME);
    return false;
  }

  if (!create_bit_pin(&probe_signal_ptr_, HAL_IN, "probe-signal-in"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize probe-signal-in pin",
                    CNAME);
    return false;
  }
  **probe_signal_ptr_ = false;  // Probe is off by default

  if (!create_bit_pin(&probe_signal_active_low_ptr_, HAL_IN,
                      "probe-signal-active-low"))
  {
    HAL_ROS_LOG_ERR(
        CNAME, "%s: Failed to initialize probe-signal-active-low pin", CNAME);
    return false;
  }
  **probe_signal_active_low_ptr_ = false;  // Probe is active high by default

  if (!create_bit_pin(&probe_out_ptr_, HAL_OUT, "probe-out"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize probe-out pin", CNAME);
    return false;
  }
  **probe_out_ptr_ = false;  // Probe is off by default

  if (!create_s32_pin(&probe_capture_ptr_, HAL_OUT, "probe-capture"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize probe-capture pin", CNAME);
    return false;
  }

  if (!create_s32_pin(&probe_transition_ptr_, HAL_OUT, "probe-transition"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize probe-transition pin",
                    CNAME);
    return false;
  }

  if (!create_s32_pin(&error_code_ptr_, HAL_OUT, "error-code"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize error-code pin", CNAME);
    return false;
  }

  if (!create_bit_pin(&estop_pin_ptr_, HAL_IN, "estop"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize estop pin", CNAME);
    return false;
  }

  if (!create_bit_pin(&stop_pin_ptr_, HAL_IO, "stop"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize stop pin", CNAME);
    return false;
  }

  if (!create_bit_pin(&safety_input_pin_ptr_, HAL_IN, "safety_input"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize safety_input pin", CNAME);
    return false;
  }

  if (!create_bit_pin(&enabling_input_pin_ptr_, HAL_IN, "enabling_input"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize enabling_input pin", CNAME);
    return false;
  }

  HAL_ROS_LOG_INFO(CNAME, "%s:  Initialized HAL pins", CNAME);

  // Export the function
  if (hal_export_functf(funct, this, 1, 0, comp_id_, "%s.funct", CNAME) < 0)
  {
    HAL_ROS_LOG_INFO(CNAME, "%s: ERROR: hal_export_functf failed", CNAME);
    hal_exit(comp_id_);
    return false;
  }
  HAL_ROS_LOG_INFO(CNAME, "%s:  Exported HAL function", CNAME);

  // Mark component ready
  hal_ready(comp_id_);

  HAL_ROS_LOG_INFO(CNAME, "%s:  HAL component ready!", CNAME);

  // Initialize miscellaneous members here
  probe_signal_ = 0;
  probe_result_type_ = 0;
  probe_event_time_ = ros::Time(0);
  probe_request_capture_type_ = 0;
  probe_result_type_ = 0;
  error_code_ = 0;

  return true;
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

bool HalHWInterface::create_s32_pin(int*** ptr, hal_pin_dir_t dir,
                                    const char* name)
{
  // Allocate space
  *ptr = ((hal_s32_t**)hal_malloc(sizeof(hal_s32_t*)));
  if (*ptr == NULL)
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Allocate HAL pin failed", CNAME);
    return false;
  }
  if (hal_pin_s32_newf(dir, *ptr, comp_id_, "%s.%s", CNAME, name))
  {
    HAL_ROS_LOG_INFO(CNAME, "%s: New HAL pin %s.%s failed", CNAME, CNAME, name);
    return false;
  }

  HAL_ROS_LOG_INFO(CNAME, "%s: New HAL pin %s.%s succeeded; addr %p", CNAME,
                   CNAME, name, *ptr);
  return true;
}

void HalHWInterface::read_with_time(ros::Duration& elapsed_time,
                                    ros::Time const& current_time,
                                    ros::Duration period)
{
  // Copy HAL joint feedback pin values to controller joint states
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    // Cache this for acceleration difference later
    joint_velocity_prev_[joint_id] = joint_velocity_[joint_id];

    joint_position_[joint_id] = **joint_pos_fb_ptrs_[joint_id];
    joint_velocity_[joint_id] = **joint_vel_fb_ptrs_[joint_id];
    joint_effort_[joint_id] = **joint_eff_fb_ptrs_[joint_id];
  }

  // Read reset pin
  reset_controllers = **reset_ptr_;

  // Emergency stop event:  pass estop HAL pin value to controller
  estop_event_ = **estop_pin_ptr_;

  // Stop event:  pass stop HAL pin value to controller
  stop_event_ = **stop_pin_ptr_;

  // Probing
  {
    // Probe signal internally is active high, converted at the pin level for
    // active-low probes
    const bool probe_active_signal =
        **probe_signal_ptr_ ^ **probe_signal_active_low_ptr_;
    const bool last_probe_active_signal = probe_signal_;

    // IMPORTANT update these first before updating the last probe signal value
    if (probe_active_signal ^ last_probe_active_signal)
    {
      probe_transition_ =
          probe_active_signal ?
              (int)machinekit_interfaces::ProbeTransitions::RISING :
              (int)machinekit_interfaces::ProbeTransitions::FALLING;
    }
    else
    {
      probe_transition_ = (int)machinekit_interfaces::ProbeTransitions::NONE;
    }
    probe_signal_ = probe_active_signal;
  }

  int expected_transition =
      (int)machinekit_interfaces::ProbeHandle::transitionNeededForCapture(
          (machinekit_interfaces::ProbeCaptureType)probe_request_capture_type_);
  if (!probe_result_type_ && probe_request_capture_type_ &&
      probe_transition_ == expected_transition)
  {
    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
    {
      // Explicitly copy elements without re-allocating
      probe_joint_position_[joint_id] = joint_position_[joint_id];
      probe_joint_velocity_[joint_id] = joint_velocity_[joint_id];
      probe_joint_effort_[joint_id] = joint_effort_[joint_id];
    }
    probe_result_type_ = probe_transition_;
    probe_event_time_ = current_time;
  }

  // No overtravel support currently

  // Safety & enabling input handling
  enabling_input_event_ = **enabling_input_pin_ptr_;
  safety_input_event_ = **safety_input_pin_ptr_;
}

void HalHWInterface::write(ros::Duration& elapsed_time)
{
  const static ros::Duration min_time(0, 1);
  double elapsed_time_seconds =
      std::max(elapsed_time.toSec(), min_time.toSec());

  // Enforce joint limits
  enforceLimits(elapsed_time);
  // Copy controller joint command values to HAL joint command pins
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    // How close did we come to the last commanded position after 1 update?
    // **joint_ferror_ptrs_[joint_id] = (**joint_pos_cmd_ptrs_[joint_id] -
    // joint_position_[joint_id]);
    // How close are we to the CURRENT commanded position? This will be nonzero
    // if feedforward isn't perfect
    **joint_ferror_ptrs_[joint_id] =
        (joint_position_command_[joint_id] - joint_position_[joint_id]);
    **joint_pos_cmd_ptrs_[joint_id] = joint_position_command_[joint_id];
    **joint_vel_cmd_ptrs_[joint_id] = joint_velocity_command_[joint_id];
    **joint_eff_cmd_ptrs_[joint_id] = joint_effort_command_[joint_id];
    // In units / sec^2
    **joint_accel_ptrs_[joint_id] =
        (joint_velocity_[joint_id] - joint_velocity_prev_[joint_id]) /
        elapsed_time_seconds;
  }

  if (probe_request_capture_type_ == probe_transition_)
  {
    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
    {
      **probe_joint_result_ptrs_[joint_id] = probe_joint_position_[joint_id];
    }
  }
  **probe_capture_ptr_ = probe_request_capture_type_;
  **probe_transition_ptr_ = probe_transition_;
  **probe_out_ptr_ = probe_signal_;

  // Set error-code HAL pin to value passed from controller
  **error_code_ptr_ = error_code_;

  // Stop event handled:  pass controller stop event value to stop HAL pin
  **stop_pin_ptr_ = stop_event_;
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

}  // namespace hal_hw_interface
