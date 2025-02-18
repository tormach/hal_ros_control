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

  if (!create_s32_pin(&total_segments_in_traj_ptr_, HAL_OUT, "total_segments_in_traj"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize total_segments_in_traj", CNAME);
    return false;
  }
  **total_segments_in_traj_ptr_ = -1;  // indicate a special counter state at startup

  if (!create_s32_pin(&current_segment_in_traj_ptr_, HAL_OUT, "current_segment_in_traj"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize current_segment_in_traj", CNAME);
    return false;
  }
  **current_segment_in_traj_ptr_ = -1;  // indicate a special counter state at startup

  if (!create_s32_pin(&move_id_ptr_, HAL_OUT, "move_id"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize move_id", CNAME);
    return false;
  }
  **move_id_ptr_ = -1;  // indicate a special move counter state at startup

  if (!create_s32_pin(&feedhold_state_ptr_, HAL_OUT, "feedhold_state"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize feedhold_state", CNAME);
    return false;
  }

  hal_s32_pin_interface_.registerHandle(machinekit_interfaces::HALS32PinHandle(
    "total_segments_in_traj", &total_segments_in_traj));


  hal_s32_pin_interface_.registerHandle(machinekit_interfaces::HALS32PinHandle(
    "current_segment_in_traj", &current_segment_in_traj));


  hal_s32_pin_interface_.registerHandle(machinekit_interfaces::HALS32PinHandle(
    "move_id", &move_id));


  hal_s32_pin_interface_.registerHandle(machinekit_interfaces::HALS32PinHandle(
    "feedhold_state", &feedhold_state));

  registerInterface(&hal_s32_pin_interface_);

  hal_bit_pin_interface_.registerHandle(
      machinekit_interfaces::HALBitPinHandle("estop", &estop_event_));
  hal_bit_pin_interface_.registerHandle(
      machinekit_interfaces::HALBitPinHandle("stop", &stop_event_));
  registerInterface(&hal_bit_pin_interface_);

  hal_bit_pin_interface_.registerHandle(machinekit_interfaces::HALBitPinHandle(
      "safety_input", &safety_input_event_));
  hal_bit_pin_interface_.registerHandle(machinekit_interfaces::HALBitPinHandle(
      "enabling_input", &enabling_input_event_));
  registerInterface(&hal_bit_pin_interface_);

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
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize enabling_input pin",
                    CNAME);
    return false;
  }

  if (!create_float_pin(&joint1_start_time_ptr_, HAL_OUT, "joint1_start_time"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint1_start_time", CNAME);
    return false;
  }

  if (!create_float_pin(&joint1_duration_ptr_, HAL_OUT, "joint1_duration"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint1_duration", CNAME);
    return false;
  }

  // initialize joint1_a
  if (!create_float_pin(&joint1_a_ptr_, HAL_OUT, "joint1_a"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint1_a", CNAME);
    return false;
  }

  // initialize joint1_b
  if (!create_float_pin(&joint1_b_ptr_, HAL_OUT, "joint1_b"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint1_b", CNAME);
    return false;
  }

  // initialize joint1_c
  if (!create_float_pin(&joint1_c_ptr_, HAL_OUT, "joint1_c"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint1_c", CNAME);
    return false;
  }

  // initialize joint1_d
  if (!create_float_pin(&joint1_d_ptr_, HAL_OUT, "joint1_d"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint1_d", CNAME);
    return false;
  }

  // initialize joint1_e
  if (!create_float_pin(&joint1_e_ptr_, HAL_OUT, "joint1_e"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint1_e", CNAME);
    return false;
  }

  // initialize joint1_f
  if (!create_float_pin(&joint1_f_ptr_, HAL_OUT, "joint1_f"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint1_f", CNAME);
    return false;
  }

  // do exact same stuff but for joint 2:
  // initialize joint2_start_time
  if (!create_float_pin(&joint2_start_time_ptr_, HAL_OUT, "joint2_start_time"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint2_start_time", CNAME);
    return false;
  }

  // initialize joint2_duration
  if (!create_float_pin(&joint2_duration_ptr_, HAL_OUT, "joint2_duration"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint2_duration", CNAME);
    return false;
  }

  // initialize joint2_a
  if (!create_float_pin(&joint2_a_ptr_, HAL_OUT, "joint2_a"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint2_a", CNAME);
    return false;
  }

  // initialize joint2_b
  if (!create_float_pin(&joint2_b_ptr_, HAL_OUT, "joint2_b"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint2_b", CNAME);
    return false;
  }

  // initialize joint2_c
  if (!create_float_pin(&joint2_c_ptr_, HAL_OUT, "joint2_c"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint2_c", CNAME);
    return false;
  }

  // initialize joint2_d
  if (!create_float_pin(&joint2_d_ptr_, HAL_OUT, "joint2_d"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint2_d", CNAME);
    return false;
  }

  // initialize joint2_e
  if (!create_float_pin(&joint2_e_ptr_, HAL_OUT, "joint2_e"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint2_e", CNAME);
    return false;
  }

  // initialize joint2_f
  if (!create_float_pin(&joint2_f_ptr_, HAL_OUT, "joint2_f"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint2_f", CNAME);
    return false;
  }

  // do exact same stuff but for joint 3:
  // initialize joint3_start_time
  if (!create_float_pin(&joint3_start_time_ptr_, HAL_OUT, "joint3_start_time"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint3_start_time", CNAME);
    return false;
  }

  // initialize joint3_duration
  if (!create_float_pin(&joint3_duration_ptr_, HAL_OUT, "joint3_duration"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint3_duration", CNAME);
    return false;
  }

  // initialize joint3_a
  if (!create_float_pin(&joint3_a_ptr_, HAL_OUT, "joint3_a"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint3_a", CNAME);
    return false;
  }

  // initialize joint3_b
  if (!create_float_pin(&joint3_b_ptr_, HAL_OUT, "joint3_b"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint3_b", CNAME);
    return false;
  }

  // initialize joint3_c
  if (!create_float_pin(&joint3_c_ptr_, HAL_OUT, "joint3_c"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint3_c", CNAME);
    return false;
  }

  // initialize joint3_d
  if (!create_float_pin(&joint3_d_ptr_, HAL_OUT, "joint3_d"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint3_d", CNAME);
    return false;
  }

  // initialize joint3_e
  if (!create_float_pin(&joint3_e_ptr_, HAL_OUT, "joint3_e"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint3_e", CNAME);
    return false;
  }

  // initialize joint3_f
  if (!create_float_pin(&joint3_f_ptr_, HAL_OUT, "joint3_f"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint3_f", CNAME);
    return false;
  }

  // initialize joint4_start_time
  if (!create_float_pin(&joint4_start_time_ptr_, HAL_OUT, "joint4_start_time"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint4_start_time", CNAME);
    return false;
  }

  // initialize joint4_duration
  if (!create_float_pin(&joint4_duration_ptr_, HAL_OUT, "joint4_duration"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint4_duration", CNAME);
    return false;
  }

  // initialize joint4_a
  if (!create_float_pin(&joint4_a_ptr_, HAL_OUT, "joint4_a"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint4_a", CNAME);
    return false;
  }

  // initialize joint4_b
  if (!create_float_pin(&joint4_b_ptr_, HAL_OUT, "joint4_b"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint4_b", CNAME);
    return false;
  }

  // initialize joint4_c
  if (!create_float_pin(&joint4_c_ptr_, HAL_OUT, "joint4_c"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint4_c", CNAME);
    return false;
  }

  // initialize joint4_d
  if (!create_float_pin(&joint4_d_ptr_, HAL_OUT, "joint4_d"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint4_d", CNAME);
    return false;
  }

  // initialize joint4_e
  if (!create_float_pin(&joint4_e_ptr_, HAL_OUT, "joint4_e"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint4_e", CNAME);
    return false;
  }

  // initialize joint4_f
  if (!create_float_pin(&joint4_f_ptr_, HAL_OUT, "joint4_f"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint4_f", CNAME);
    return false;
  }

  // initialize joint5_start_time
  if (!create_float_pin(&joint5_start_time_ptr_, HAL_OUT, "joint5_start_time"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint5_start_time", CNAME);
    return false;
  }

  // initialize joint5_duration
  if (!create_float_pin(&joint5_duration_ptr_, HAL_OUT, "joint5_duration"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint5_duration", CNAME);
    return false;
  }

  // initialize joint5_a
  if (!create_float_pin(&joint5_a_ptr_, HAL_OUT, "joint5_a"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint5_a", CNAME);
    return false;
  }

  // initialize joint5_b
  if (!create_float_pin(&joint5_b_ptr_, HAL_OUT, "joint5_b"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint5_b", CNAME);
    return false;
  }

  // initialize joint5_c
  if (!create_float_pin(&joint5_c_ptr_, HAL_OUT, "joint5_c"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint5_c", CNAME);
    return false;
  }

  // initialize joint5_d
  if (!create_float_pin(&joint5_d_ptr_, HAL_OUT, "joint5_d"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint5_d", CNAME);
    return false;
  }

  // initialize joint5_e
  if (!create_float_pin(&joint5_e_ptr_, HAL_OUT, "joint5_e"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint5_e", CNAME);
    return false;
  }

  // initialize joint5_f
  if (!create_float_pin(&joint5_f_ptr_, HAL_OUT, "joint5_f"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint5_f", CNAME);
    return false;
  }

  // initialize joint6_start_time
  if (!create_float_pin(&joint6_start_time_ptr_, HAL_OUT, "joint6_start_time"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint6_start_time", CNAME);
    return false;
  }

  // initialize joint6_duration
  if (!create_float_pin(&joint6_duration_ptr_, HAL_OUT, "joint6_duration"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint6_duration", CNAME);
    return false;
  }

  // initialize joint6_a
  if (!create_float_pin(&joint6_a_ptr_, HAL_OUT, "joint6_a"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint6_a", CNAME);
    return false;
  }
  // initialize joint6_b
  if (!create_float_pin(&joint6_b_ptr_, HAL_OUT, "joint6_b"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint6_b", CNAME);
    return false;
  }

  // initialize joint6_c
  if (!create_float_pin(&joint6_c_ptr_, HAL_OUT, "joint6_c"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint6_c", CNAME);
    return false;
  }

  // initialize joint6_d
  if (!create_float_pin(&joint6_d_ptr_, HAL_OUT, "joint6_d"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint6_d", CNAME);
    return false;
  }

  // initialize joint6_e
  if (!create_float_pin(&joint6_e_ptr_, HAL_OUT, "joint6_e"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint6_e", CNAME);
    return false;
  }

  // initialize joint6_f
  if (!create_float_pin(&joint6_f_ptr_, HAL_OUT, "joint6_f"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize joint6_f", CNAME);
    return false;
  }

  // TODO: INITIALIZE NEW STUFF

  if (!create_float_pin(&elapsed_trajectory_time_ptr_, HAL_OUT, "elapsed_trajectory_time"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize elapsed_trajectory_time", CNAME);
    return false;
  }

  if (!create_float_pin(&absolute_time_ptr_, HAL_OUT, "absolute_time"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize absolute_time", CNAME);
    return false;
  }

  if (!create_float_pin(&velocity_scale_ptr_, HAL_OUT, "velocity_scale"))
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Failed to initialize velocity_scale", CNAME);
    return false;
  }

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint1_start_time",
                                                  &joint1_start_time_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint1_duration",
                                                  &joint1_duration_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint1_a", &joint1_a_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint1_b", &joint1_b_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint1_c", &joint1_c_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint1_d", &joint1_d_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint1_e", &joint1_e_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint1_f", &joint1_f_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint2_start_time",
                                                  &joint2_start_time_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint2_duration",
                                                  &joint2_duration_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint2_a", &joint2_a_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint2_b", &joint2_b_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint2_c", &joint2_c_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint2_d", &joint2_d_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint2_e", &joint2_e_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint2_f", &joint2_f_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint3_start_time",
                                                  &joint3_start_time_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint3_duration",
                                                  &joint3_duration_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint3_a", &joint3_a_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint3_b", &joint3_b_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint3_c", &joint3_c_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint3_d", &joint3_d_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint3_e", &joint3_e_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint3_f", &joint3_f_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint4_start_time",
                                                  &joint4_start_time_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint4_duration",
                                                  &joint4_duration_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint4_a", &joint4_a_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint4_b", &joint4_b_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint4_c", &joint4_c_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint4_d", &joint4_d_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint4_e", &joint4_e_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint4_f", &joint4_f_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint5_start_time",
                                                  &joint5_start_time_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint5_duration",
                                                  &joint5_duration_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint5_a", &joint5_a_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint5_b", &joint5_b_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint5_c", &joint5_c_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint5_d", &joint5_d_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint5_e", &joint5_e_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint5_f", &joint5_f_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint6_start_time",
                                                  &joint6_start_time_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint6_duration",
                                                  &joint6_duration_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint6_a", &joint6_a_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint6_b", &joint6_b_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint6_c", &joint6_c_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint6_d", &joint6_d_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint6_e", &joint6_e_));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("joint6_f", &joint6_f_));



  // TODO: INITIALIZE NEW STUFF

  // TODO: REGISTER NEW HANDLES

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("elapsed_trajectory_time", &elapsed_trajectory_time));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("absolute_time", &absolute_time));

  hal_float_pin_interface_.registerHandle(
      machinekit_interfaces::HALPinHandle<double>("velocity_scale", &velocity_scale));

  registerInterface(&hal_float_pin_interface_);



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

// First add this helper function alongside other create_*_pin functions
bool HalHWInterface::create_float_pin(double*** ptr, hal_pin_dir_t dir,
                                      const char* name)
{
  // Allocate space
  *ptr = ((hal_float_t**)hal_malloc(sizeof(hal_float_t*)));
  if (*ptr == NULL)
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: Allocate HAL pin failed", CNAME);
    return false;
  }
  if (hal_pin_float_newf(dir, *ptr, comp_id_, "%s.%s", CNAME, name))
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

  **joint1_start_time_ptr_ = joint1_start_time_;
  **joint1_duration_ptr_ = joint1_duration_;
  **joint1_a_ptr_ = joint1_a_;
  **joint1_b_ptr_ = joint1_b_;
  **joint1_c_ptr_ = joint1_c_;
  **joint1_d_ptr_ = joint1_d_;
  **joint1_e_ptr_ = joint1_e_;
  **joint1_f_ptr_ = joint1_f_;

  **joint2_start_time_ptr_ = joint2_start_time_;
  **joint2_duration_ptr_ = joint2_duration_;
  **joint2_a_ptr_ = joint2_a_;
  **joint2_b_ptr_ = joint2_b_;
  **joint2_c_ptr_ = joint2_c_;
  **joint2_d_ptr_ = joint2_d_;
  **joint2_e_ptr_ = joint2_e_;
  **joint2_f_ptr_ = joint2_f_;

  **joint3_start_time_ptr_ = joint3_start_time_;
  **joint3_duration_ptr_ = joint3_duration_;
  **joint3_a_ptr_ = joint3_a_;
  **joint3_b_ptr_ = joint3_b_;
  **joint3_c_ptr_ = joint3_c_;
  **joint3_d_ptr_ = joint3_d_;
  **joint3_e_ptr_ = joint3_e_;
  **joint3_f_ptr_ = joint3_f_;

  **joint4_start_time_ptr_ = joint4_start_time_;
  **joint4_duration_ptr_ = joint4_duration_;
  **joint4_a_ptr_ = joint4_a_;
  **joint4_b_ptr_ = joint4_b_;
  **joint4_c_ptr_ = joint4_c_;
  **joint4_d_ptr_ = joint4_d_;
  **joint4_e_ptr_ = joint4_e_;
  **joint4_f_ptr_ = joint4_f_;

  **joint5_start_time_ptr_ = joint5_start_time_;
  **joint5_duration_ptr_ = joint5_duration_;
  **joint5_a_ptr_ = joint5_a_;
  **joint5_b_ptr_ = joint5_b_;
  **joint5_c_ptr_ = joint5_c_;
  **joint5_d_ptr_ = joint5_d_;
  **joint5_e_ptr_ = joint5_e_;
  **joint5_f_ptr_ = joint5_f_;

  **joint6_start_time_ptr_ = joint6_start_time_;
  **joint6_duration_ptr_ = joint6_duration_;
  **joint6_a_ptr_ = joint6_a_;
  **joint6_b_ptr_ = joint6_b_;
  **joint6_c_ptr_ = joint6_c_;
  **joint6_d_ptr_ = joint6_d_;
  **joint6_e_ptr_ = joint6_e_;
  **joint6_f_ptr_ = joint6_f_;

  **total_segments_in_traj_ptr_ = total_segments_in_traj;
  **current_segment_in_traj_ptr_ = current_segment_in_traj;
  **move_id_ptr_ = move_id;
  **feedhold_state_ptr_ = feedhold_state;

  **elapsed_trajectory_time_ptr_ = elapsed_trajectory_time;
  **velocity_scale_ptr_ = velocity_scale;

  **absolute_time_ptr_ = absolute_time;
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
