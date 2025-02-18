///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2020, Tormach, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Robert W. Ellenberg

#pragma once

// C++ standard
#include <cassert>
#include <stdexcept>
#include <string>
#include <memory>
#include <iomanip>
#include <cmath>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/dynamic_bitset.hpp>

// ROS
#include <ros/node_handle.h>

// URDF
#include <urdf/model.h>

// ROS messages
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <stop_event_msgs/SetNextProbeMove.h>
#include <stop_event_msgs/GetStopEventResult.h>
#include <stop_event_msgs/GetJointTrajectoryErrorContext.h>

// ros_controls
#include <realtime_tools/realtime_server_goal_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/internal/demangle_symbol.h>

#include <machinekit_interfaces/realtime_event_interface.h>
#include <machinekit_interfaces/probe_interface.h>
#include <machinekit_interfaces/joint_event_interface.h>
#include <machinekit_interfaces/hal_pin_interface.h>

// For reading and writing the scalling factor
// and spinning a separate thread with ros publisher interface
// to offload controller `update` method
#include <redis_store_msgs/ParamUpdate.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <std_srvs/Trigger.h>
#include <realtime_tools/realtime_buffer.h>

// Bring in enums
using machinekit_interfaces::ProbeState;
using machinekit_interfaces::ProbeTransitions;
using stop_event_msgs::GetJointTrajectoryErrorContextResponse;
using stop_event_msgs::SetNextProbeMoveRequest;
using stop_event_msgs::SetNextProbeMoveResponse;

// Project
#include <joint_trajectory_controller/joint_trajectory_controller.h>

// Dynamic Velocity Scale
#include "interruptible_joint_trajectory_controller/velocity_scale_manager.h"

// Separate thread handling updated velocity scale publishing
#include "interruptible_joint_trajectory_controller/CommThread.h"

// Trajectory recording experiments
#include <functional>
#include <numeric>
#include <joint_trajectory_controller/joint_trajectory_segment.h>

namespace interruptible_joint_trajectory_controller
{
static const std::string PROBE_SERVICE_NAME{ "probe" };
static const std::string PROBE_RESULT_SERVICE_NAME{ "probe_result" };
static const std::string ERROR_CONTEXT_SERVICE_NAME{ "error_context" };

struct ProbeSettings
{
  int probe_request_capture_type;
};

/**
 * Controller for executing joint-space trajectories on a group of joints, that
 * can respond to stop events triggered from hardware in the realtime loop.
 *
 * See JointTrajectoryController documentation for details on how the trajectory
 * execution works.
 *
 */

template <class SegmentImpl, class HardwareInterface>
class InterruptibleJointTrajectoryController
  : public joint_trajectory_controller::JointTrajectoryController<
        SegmentImpl, HardwareInterface, ProbeSettings>
{
protected:
  using JointTrajectorySegmentType =
      joint_trajectory_controller::JointTrajectorySegment<SegmentImpl>;
  using JointTrajectoryControllerType =
      typename joint_trajectory_controller::JointTrajectoryController<
          SegmentImpl, HardwareInterface, ProbeSettings>;
  using typename JointTrajectoryControllerType::ExtendedTrajectoryPtr;
  using typename JointTrajectoryControllerType::JointTrajectoryConstPtr;
  using typename JointTrajectoryControllerType::RealtimeGoalHandlePtr;
  using typename JointTrajectoryControllerType::Trajectory;



public:
  InterruptibleJointTrajectoryController();

  /** \name Non Real-Time Safe Functions
   *\{*/

  // KLUDGE have to override this method in Controller to be able to initialize
  // multiple hardware interface types
  bool initRequest(hardware_interface::RobotHW* robot_hw,
                   ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh,
                   controller_interface::ControllerBase::ClaimedResources&
                       claimed_resources) override;
  /*\}*/

protected:
  void update(const ros::Time& time, const ros::Duration& period);
  void handle_estop_event(
      ExtendedTrajectoryPtr, joint_trajectory_controller::TimeData,
      typename JointTrajectoryControllerType::RealtimeGoalHandlePtr);
  void handle_stop_event(
      ExtendedTrajectoryPtr, joint_trajectory_controller::TimeData,
      typename JointTrajectoryControllerType::RealtimeGoalHandlePtr);

  void handle_probe_transitions(
      ExtendedTrajectoryPtr, joint_trajectory_controller::TimeData,
      typename JointTrajectoryControllerType::RealtimeGoalHandlePtr);
  virtual void onTrajectoryError(int error_code);
  /*\}*/

  // Template for claiming a hardware interface of some type, not RT safe
  template <typename intf_type, typename handle_type>
  inline bool claim_hardware_resources(
      hardware_interface::RobotHW*,
      controller_interface::ControllerBase::ClaimedResources&,
      std::vector<handle_type>&, const std::vector<std::string>&);

  // Same for one item
  template <typename intf_type, typename handle_type>
  inline bool claim_hardware_resources(
      hardware_interface::RobotHW*,
      controller_interface::ControllerBase::ClaimedResources&, handle_type&,
      const std::string);

  // Command handling, not real-time safe
  virtual bool updateTrajectoryCommand(const JointTrajectoryConstPtr& msg,
                                       RealtimeGoalHandlePtr gh,
                                       std::string* error_string = nullptr);

  // Service calls, not real-time safe

  /**
   * @brief Service callback to force the controller into the hold position.
   *
   * @param request Dummy for triggering the service
   * @param response True on success.
   *
   * @return False if something went wrong. True otherwise.
   */
  bool handleProbeRequest(stop_event_msgs::SetNextProbeMoveRequest& request,
                          stop_event_msgs::SetNextProbeMoveResponse& response);
  bool handleStopEventResultRequest(
      stop_event_msgs::GetStopEventResultRequest& request,
      stop_event_msgs::GetStopEventResultResponse& response);
  bool handleJointTrajectoryErrorContextRequest(
      stop_event_msgs::GetJointTrajectoryErrorContextRequest& request,
      stop_event_msgs::GetJointTrajectoryErrorContextResponse& response);

  // Real-Time ONLY Functions (must be called within the context of an update)
  virtual void checkReachedTrajectoryGoal(ros::Time const& uptime);
  virtual void checkReachedTrajectoryGoalProbe(int capture_type,
                                               ros::Time const& uptime);

  // Services for controller trajectory behavior
  ros::ServiceServer probe_service_;  //!< Declare success when probe trip
                                      //!< occurs on the next send trajectory
  ros::ServiceServer probe_result_service_;  //!< Request the result of a probe
  ros::ServiceServer error_detail_service_;  //!< Used to query verbose error
                                             //!< data after a motion error has
                                             //!< occurred

  boost::shared_ptr<VelocityScaleManager> velocity_scale_manager_;
  bool safety_input_previous_state_;
  boost::shared_ptr<CommThread> comm_thread_;  // Use shared_ptr for automatic
                                               // memory management
  double scale_factor_before_safety_trip_;

  std::vector<const JointTrajectorySegmentType*> current_segments_;

  std::vector<machinekit_interfaces::JointEventDataHandle> probe_joint_results_;
  machinekit_interfaces::ProbeHandle probe_handle_;
  machinekit_interfaces::RealtimeEventHandle stop_event_;
  machinekit_interfaces::HALS32PinHandle error_code_;
  int jog_err_threshold_;
  int jog_err_count_;  // Used to count "soft" errors that should only be
                       // reported if many happen at once
  machinekit_interfaces::HALBitPinHandle stop_handle_;
  machinekit_interfaces::HALBitPinHandle estop_handle_;

  machinekit_interfaces::HALBitPinHandle safety_input_handle_;
  machinekit_interfaces::HALBitPinHandle enabling_input_handle_;

  machinekit_interfaces::HALPinHandle<double> joint1_start_time_handle_;
  machinekit_interfaces::HALPinHandle<double> joint1_duration_handle_;
  machinekit_interfaces::HALPinHandle<double> joint1_a_handle_;
  machinekit_interfaces::HALPinHandle<double> joint1_b_handle_;
  machinekit_interfaces::HALPinHandle<double> joint1_c_handle_;
  machinekit_interfaces::HALPinHandle<double> joint1_d_handle_;
  machinekit_interfaces::HALPinHandle<double> joint1_e_handle_;
  machinekit_interfaces::HALPinHandle<double> joint1_f_handle_;

  machinekit_interfaces::HALPinHandle<double> joint2_start_time_handle_;
  machinekit_interfaces::HALPinHandle<double> joint2_duration_handle_;
  machinekit_interfaces::HALPinHandle<double> joint2_a_handle_;
  machinekit_interfaces::HALPinHandle<double> joint2_b_handle_;
  machinekit_interfaces::HALPinHandle<double> joint2_c_handle_;
  machinekit_interfaces::HALPinHandle<double> joint2_d_handle_;
  machinekit_interfaces::HALPinHandle<double> joint2_e_handle_;
  machinekit_interfaces::HALPinHandle<double> joint2_f_handle_;

  machinekit_interfaces::HALPinHandle<double> joint3_start_time_handle_;
  machinekit_interfaces::HALPinHandle<double> joint3_duration_handle_;
  machinekit_interfaces::HALPinHandle<double> joint3_a_handle_;
  machinekit_interfaces::HALPinHandle<double> joint3_b_handle_;
  machinekit_interfaces::HALPinHandle<double> joint3_c_handle_;
  machinekit_interfaces::HALPinHandle<double> joint3_d_handle_;
  machinekit_interfaces::HALPinHandle<double> joint3_e_handle_;
  machinekit_interfaces::HALPinHandle<double> joint3_f_handle_;

  machinekit_interfaces::HALPinHandle<double> joint4_start_time_handle_;
  machinekit_interfaces::HALPinHandle<double> joint4_duration_handle_;
  machinekit_interfaces::HALPinHandle<double> joint4_a_handle_;
  machinekit_interfaces::HALPinHandle<double> joint4_b_handle_;
  machinekit_interfaces::HALPinHandle<double> joint4_c_handle_;
  machinekit_interfaces::HALPinHandle<double> joint4_d_handle_;
  machinekit_interfaces::HALPinHandle<double> joint4_e_handle_;
  machinekit_interfaces::HALPinHandle<double> joint4_f_handle_;

  machinekit_interfaces::HALPinHandle<double> joint5_start_time_handle_;
  machinekit_interfaces::HALPinHandle<double> joint5_duration_handle_;
  machinekit_interfaces::HALPinHandle<double> joint5_a_handle_;
  machinekit_interfaces::HALPinHandle<double> joint5_b_handle_;
  machinekit_interfaces::HALPinHandle<double> joint5_c_handle_;
  machinekit_interfaces::HALPinHandle<double> joint5_d_handle_;
  machinekit_interfaces::HALPinHandle<double> joint5_e_handle_;
  machinekit_interfaces::HALPinHandle<double> joint5_f_handle_;

  machinekit_interfaces::HALPinHandle<double> joint6_start_time_handle_;
  machinekit_interfaces::HALPinHandle<double> joint6_duration_handle_;
  machinekit_interfaces::HALPinHandle<double> joint6_a_handle_;
  machinekit_interfaces::HALPinHandle<double> joint6_b_handle_;
  machinekit_interfaces::HALPinHandle<double> joint6_c_handle_;
  machinekit_interfaces::HALPinHandle<double> joint6_d_handle_;
  machinekit_interfaces::HALPinHandle<double> joint6_e_handle_;
  machinekit_interfaces::HALPinHandle<double> joint6_f_handle_;


  machinekit_interfaces::HALS32PinHandle move_id_handle_;
  machinekit_interfaces::HALS32PinHandle feedhold_state_handle_;
  machinekit_interfaces::HALS32PinHandle total_segments_in_traj_handle_;
  machinekit_interfaces::HALS32PinHandle current_segment_in_traj_handle_;

machinekit_interfaces::HALPinHandle<double> velocity_scale_handle_;
machinekit_interfaces::HALPinHandle<double> elapsed_trajectory_time_handle_;
machinekit_interfaces::HALPinHandle<double> absolute_time_handle_;

  // Move ID tracking
  int move_id;
  int** move_id_ptr_;

  // Feedhold state tracking
  int feedhold_state;
  int** feedhold_state_ptr_;

  // Trajectory segment tracking
  int total_segments_in_traj;
  int** total_segments_in_traj_ptr_;

  int current_segment_in_traj;
  int** current_segment_in_traj_ptr_;
};

}  // namespace interruptible_joint_trajectory_controller

#pragma once

namespace interruptible_joint_trajectory_controller
{
template <class SegmentImpl, class HardwareInterface>
InterruptibleJointTrajectoryController<
    SegmentImpl, HardwareInterface>::InterruptibleJointTrajectoryController()
  : JointTrajectoryControllerType()
{
}

// This is following the same basic pattern as Controller<>'s initRequest, but
// we can't just use the basic init function because it's tied to the Joint
// interface. initRequest itself has to be overridden to handle these
// auxiliary interfaces.
template <class SegmentImpl, class HardwareInterface>
template <typename intf_type, typename handle_type>
inline bool
InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
    claim_hardware_resources(
        hardware_interface::RobotHW* robot_hw,
        controller_interface::ControllerBase::ClaimedResources&
            claimed_resources,
        std::vector<handle_type>& handles,
        const std::vector<std::string>& names)
{
  std::size_t size = names.size();
  auto intf_typename =
      hardware_interface::internal::demangledTypeName<intf_type>();
  ROS_INFO_NAMED(this->name_, "Claiming %zu hardware resource(s) of type %s",
                 size, intf_typename.c_str());
  intf_type* intf = robot_hw->get<intf_type>();
  if (!intf)
  {
    ROS_ERROR("This controller requires a hardware interface of type '%s'."
              " Make sure this is registered in the "
              "hardware_interface::RobotHW class.",
              intf_typename.c_str());
    return false;
  }
  intf->clearClaims();

  handles.resize(size);
  for (std::size_t i = 0; i < size; i++)
  {
    auto name = names[i];
    try
    {
      handles[i] = intf->getHandle(name);
      ROS_INFO_STREAM_NAMED(this->name_, "Found hardware interface '"
                                             << name << "' of type '"
                                             << intf_typename << "'.");
    }
    catch (...)
    {
      ROS_ERROR_STREAM_NAMED(this->name_, "No hardware interface '"
                                              << name << "' of type '"
                                              << intf_typename
                                              << "' found in RobotHW class.");
      return false;
    }
  }

  hardware_interface::InterfaceResources iface_res(intf_typename,
                                                   intf->getClaims());
  claimed_resources.push_back(iface_res);
  intf->clearClaims();
  return true;
}

template <class SegmentImpl, class HardwareInterface>
template <typename intf_type, typename handle_type>
inline bool
InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
    claim_hardware_resources(
        hardware_interface::RobotHW* robot_hw,
        controller_interface::ControllerBase::ClaimedResources&
            claimed_resources,
        handle_type& handle, const std::string name)
{
  std::vector<handle_type> handles = { handle };
  const std::vector<std::string> names = { name };
  if (!claim_hardware_resources<intf_type, handle_type>(
          robot_hw, claimed_resources, handles, names))
    return false;
  handle = handles[0];
  return true;
}

template <class SegmentImpl, class HardwareInterface>
bool InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
    initRequest(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                ros::NodeHandle& controller_nh,
                controller_interface::ControllerBase::ClaimedResources&
                    claimed_resources)
{
  // Initialize auxiliary hardware interfaces required by this controller (probe
  // / stop event interfaces)
  ROS_INFO_STREAM("initRequest for InterruptibleJointTrajectoryController");

  ROS_WARN("CUSTOMCTRL");

  // SO ugly, need to redirect to the base class method, but this is fragile if
  // JointTrajectoryController ever decides to add one... Complete the
  // underlying initialization for the controller (JointTrajectoryController
  // base-level init)
  bool base_init = JointTrajectoryControllerType::initRequest(
      robot_hw, root_nh, controller_nh, claimed_resources);
  if (!base_init)
  {
    return false;
  }

  if (!claim_hardware_resources<machinekit_interfaces::ProbeInterface,
                                machinekit_interfaces::ProbeHandle>(
          robot_hw, claimed_resources, probe_handle_, "probe"))
    return false;

  // Probe results are a parallel set of handles defined by the joint names (to
  // avoid conflicts with the standard joint handles)
  std::vector<std::string> n = this->joint_names_;
  std::for_each(n.begin(), n.end(), [](auto& s) { s.append("_probe"); });
  const std::vector<std::string> joint_probe_names = n;
  if (!claim_hardware_resources<machinekit_interfaces::JointEventDataInterface,
                                machinekit_interfaces::JointEventDataHandle>(
          robot_hw, claimed_resources, probe_joint_results_, joint_probe_names))
    return false;

  if (!claim_hardware_resources<machinekit_interfaces::HALS32PinInterface,
                                machinekit_interfaces::HALS32PinHandle>(
          robot_hw, claimed_resources, error_code_, "controller_status"))
    return false;

  std::vector<machinekit_interfaces::HALBitPinHandle> bit_rsrc_handles;
  const std::vector<std::string> bit_rsrc_names = { "stop", "estop" };
  if (!claim_hardware_resources<machinekit_interfaces::HALBitPinInterface,
                                machinekit_interfaces::HALBitPinHandle>(
          robot_hw, claimed_resources, bit_rsrc_handles, bit_rsrc_names))
    return false;
  stop_handle_ = bit_rsrc_handles[0];
  estop_handle_ = bit_rsrc_handles[1];

  std::vector<machinekit_interfaces::HALBitPinHandle> bit_rsrc_handles_safety;
  const std::vector<std::string> bit_rsrc_names_safety = { "safety_input",
                                                           "enabling_input" };
  if (!claim_hardware_resources<machinekit_interfaces::HALBitPinInterface,
                                machinekit_interfaces::HALBitPinHandle>(
          robot_hw, claimed_resources, bit_rsrc_handles_safety,
          bit_rsrc_names_safety))
    return false;
  safety_input_handle_ = bit_rsrc_handles_safety[0];
  enabling_input_handle_ = bit_rsrc_handles_safety[1];

  std::vector<machinekit_interfaces::HALPinHandle<double>>
      joint1_float_rsrc_handles;
  const std::vector<std::string> joint1_float_rsrc_names = {
    "joint1_start_time", "joint1_duration", "joint1_a", "joint1_b",
    "joint1_c",          "joint1_d",        "joint1_e", "joint1_f"
  };

  if (!claim_hardware_resources<hardware_interface::HardwareResourceManager<
                                    machinekit_interfaces::HALPinHandle<double>,
                                    hardware_interface::DontClaimResources>,
                                machinekit_interfaces::HALPinHandle<double>>(
          robot_hw, claimed_resources, joint1_float_rsrc_handles,
          joint1_float_rsrc_names))
    return false;

  joint1_start_time_handle_ = joint1_float_rsrc_handles[0];
  joint1_duration_handle_ = joint1_float_rsrc_handles[1];
  joint1_a_handle_ = joint1_float_rsrc_handles[2];
  joint1_b_handle_ = joint1_float_rsrc_handles[3];
  joint1_c_handle_ = joint1_float_rsrc_handles[4];
  joint1_d_handle_ = joint1_float_rsrc_handles[5];
  joint1_e_handle_ = joint1_float_rsrc_handles[6];
  joint1_f_handle_ = joint1_float_rsrc_handles[7];

  // Joint 2
  std::vector<machinekit_interfaces::HALPinHandle<double>>
      joint2_float_rsrc_handles;
  const std::vector<std::string> joint2_float_rsrc_names = {
    "joint2_start_time", "joint2_duration", "joint2_a", "joint2_b",
    "joint2_c",          "joint2_d",        "joint2_e", "joint2_f"
  };

  if (!claim_hardware_resources<hardware_interface::HardwareResourceManager<
                                    machinekit_interfaces::HALPinHandle<double>,
                                    hardware_interface::DontClaimResources>,
                                machinekit_interfaces::HALPinHandle<double>>(
          robot_hw, claimed_resources, joint2_float_rsrc_handles,
          joint2_float_rsrc_names))
    return false;

  joint2_start_time_handle_ = joint2_float_rsrc_handles[0];
  joint2_duration_handle_ = joint2_float_rsrc_handles[1];
  joint2_a_handle_ = joint2_float_rsrc_handles[2];
  joint2_b_handle_ = joint2_float_rsrc_handles[3];
  joint2_c_handle_ = joint2_float_rsrc_handles[4];
  joint2_d_handle_ = joint2_float_rsrc_handles[5];
  joint2_e_handle_ = joint2_float_rsrc_handles[6];
  joint2_f_handle_ = joint2_float_rsrc_handles[7];

  // Joint 3
  std::vector<machinekit_interfaces::HALPinHandle<double>>
      joint3_float_rsrc_handles;
  const std::vector<std::string> joint3_float_rsrc_names = {
    "joint3_start_time", "joint3_duration", "joint3_a", "joint3_b",
    "joint3_c",          "joint3_d",        "joint3_e", "joint3_f"
  };

  if (!claim_hardware_resources<hardware_interface::HardwareResourceManager<
                                    machinekit_interfaces::HALPinHandle<double>,
                                    hardware_interface::DontClaimResources>,
                                machinekit_interfaces::HALPinHandle<double>>(
          robot_hw, claimed_resources, joint3_float_rsrc_handles,
          joint3_float_rsrc_names))
    return false;

  joint3_start_time_handle_ = joint3_float_rsrc_handles[0];
  joint3_duration_handle_ = joint3_float_rsrc_handles[1];
  joint3_a_handle_ = joint3_float_rsrc_handles[2];
  joint3_b_handle_ = joint3_float_rsrc_handles[3];
  joint3_c_handle_ = joint3_float_rsrc_handles[4];
  joint3_d_handle_ = joint3_float_rsrc_handles[5];
  joint3_e_handle_ = joint3_float_rsrc_handles[6];
  joint3_f_handle_ = joint3_float_rsrc_handles[7];

  // Joint 4
  std::vector<machinekit_interfaces::HALPinHandle<double>>
      joint4_float_rsrc_handles;
  const std::vector<std::string> joint4_float_rsrc_names = {
    "joint4_start_time", "joint4_duration", "joint4_a", "joint4_b",
    "joint4_c",          "joint4_d",        "joint4_e", "joint4_f"
  };

  if (!claim_hardware_resources<hardware_interface::HardwareResourceManager<
                                    machinekit_interfaces::HALPinHandle<double>,
                                    hardware_interface::DontClaimResources>,
                                machinekit_interfaces::HALPinHandle<double>>(
          robot_hw, claimed_resources, joint4_float_rsrc_handles,
          joint4_float_rsrc_names))
    return false;

  joint4_start_time_handle_ = joint4_float_rsrc_handles[0];
  joint4_duration_handle_ = joint4_float_rsrc_handles[1];
  joint4_a_handle_ = joint4_float_rsrc_handles[2];
  joint4_b_handle_ = joint4_float_rsrc_handles[3];
  joint4_c_handle_ = joint4_float_rsrc_handles[4];
  joint4_d_handle_ = joint4_float_rsrc_handles[5];
  joint4_e_handle_ = joint4_float_rsrc_handles[6];
  joint4_f_handle_ = joint4_float_rsrc_handles[7];

  // Joint 5
  std::vector<machinekit_interfaces::HALPinHandle<double>>
      joint5_float_rsrc_handles;
  const std::vector<std::string> joint5_float_rsrc_names = {
    "joint5_start_time", "joint5_duration", "joint5_a", "joint5_b",
    "joint5_c",          "joint5_d",        "joint5_e", "joint5_f"
  };

  if (!claim_hardware_resources<hardware_interface::HardwareResourceManager<
                                    machinekit_interfaces::HALPinHandle<double>,
                                    hardware_interface::DontClaimResources>,
                                machinekit_interfaces::HALPinHandle<double>>(
          robot_hw, claimed_resources, joint5_float_rsrc_handles,
          joint5_float_rsrc_names))
    return false;

  joint5_start_time_handle_ = joint5_float_rsrc_handles[0];
  joint5_duration_handle_ = joint5_float_rsrc_handles[1];
  joint5_a_handle_ = joint5_float_rsrc_handles[2];
  joint5_b_handle_ = joint5_float_rsrc_handles[3];
  joint5_c_handle_ = joint5_float_rsrc_handles[4];
  joint5_d_handle_ = joint5_float_rsrc_handles[5];
  joint5_e_handle_ = joint5_float_rsrc_handles[6];
  joint5_f_handle_ = joint5_float_rsrc_handles[7];

  // Joint 6
  std::vector<machinekit_interfaces::HALPinHandle<double>>
      joint6_float_rsrc_handles;
  const std::vector<std::string> joint6_float_rsrc_names = {
    "joint6_start_time", "joint6_duration", "joint6_a", "joint6_b",
    "joint6_c",          "joint6_d",        "joint6_e", "joint6_f"
  };

  if (!claim_hardware_resources<hardware_interface::HardwareResourceManager<
                                    machinekit_interfaces::HALPinHandle<double>,
                                    hardware_interface::DontClaimResources>,
                                machinekit_interfaces::HALPinHandle<double>>(
          robot_hw, claimed_resources, joint6_float_rsrc_handles,
          joint6_float_rsrc_names))
    return false;

  joint6_start_time_handle_ = joint6_float_rsrc_handles[0];
  joint6_duration_handle_ = joint6_float_rsrc_handles[1];
  joint6_a_handle_ = joint6_float_rsrc_handles[2];
  joint6_b_handle_ = joint6_float_rsrc_handles[3];
  joint6_c_handle_ = joint6_float_rsrc_handles[4];
  joint6_d_handle_ = joint6_float_rsrc_handles[5];
  joint6_e_handle_ = joint6_float_rsrc_handles[6];
  joint6_f_handle_ = joint6_float_rsrc_handles[7];

  // TODO: Common Extra float pins

  std::vector<machinekit_interfaces::HALPinHandle<double>>
      common_extra_float_rsrc_handles;
  const std::vector<std::string> common_extra_float_rsrc_names = {
    "velocity_scale", "elapsed_trajectory_time", "absolute_time"
  };

  if (!claim_hardware_resources<hardware_interface::HardwareResourceManager<
                                    machinekit_interfaces::HALPinHandle<double>,
                                    hardware_interface::DontClaimResources>,
                                machinekit_interfaces::HALPinHandle<double>>(
          robot_hw, claimed_resources, common_extra_float_rsrc_handles,
          common_extra_float_rsrc_names))
    return false;

  velocity_scale_handle_ = common_extra_float_rsrc_handles[0];
  elapsed_trajectory_time_handle_ = common_extra_float_rsrc_handles[1];
  absolute_time_handle_ = common_extra_float_rsrc_handles[2];


  ROS_INFO_STREAM_NAMED(this->name_, "Claimed " << claimed_resources.size()
                                                << " hardware interface types");
  for (auto const& s : claimed_resources)
  {
    for (auto const& c : s.resources)
    {
      ROS_INFO_STREAM_NAMED(this->name_, "interface " << s.hardware_interface
                                                      << " claims " << c);
    }
  }
  ROS_INFO_STREAM_NAMED(this->name_, "Starting probe services");
  // Set up services to control probe behavior
  probe_service_ = controller_nh.advertiseService(
      PROBE_SERVICE_NAME,
      &InterruptibleJointTrajectoryController::handleProbeRequest, this);
  probe_result_service_ = controller_nh.advertiseService(
      PROBE_RESULT_SERVICE_NAME,
      &InterruptibleJointTrajectoryController::handleStopEventResultRequest,
      this);
  error_detail_service_ = controller_nh.advertiseService(
      ERROR_CONTEXT_SERVICE_NAME,
      &InterruptibleJointTrajectoryController::
          handleJointTrajectoryErrorContextRequest,
      this);

  boost::shared_ptr<ros::NodeHandle> nh_ptr2 =
      boost::make_shared<ros::NodeHandle>(root_nh);
  velocity_scale_manager_ = boost::make_shared<VelocityScaleManager>(nh_ptr2);
  comm_thread_ = boost::make_shared<CommThread>();
  comm_thread_->start();

  safety_input_previous_state_ = true;

  // KLUDGE soft error threshold so jogging with probe active doesn't spam the
  // console
  jog_err_threshold_ = 32;
  this->controller_nh_.getParam("jog_error_threshold", jog_err_threshold_);
  jog_err_count_ = -1;
  // success
  this->state_ = controller_interface::Controller<
      HardwareInterface>::ControllerState::INITIALIZED;

  // Get the HALS32PinInterface
  auto* s32_pin_interface = robot_hw->get<machinekit_interfaces::HALS32PinInterface>();
  if (!s32_pin_interface) {
      ROS_ERROR("Failed to get HALS32PinInterface");
      return false;
  }

  // Set up the vector of pin handles and names
  std::vector<machinekit_interfaces::HALS32PinHandle> s32_rsrc_handles;
  const std::vector<std::string> s32_rsrc_names = {
      "move_id", 
      "feedhold_state",
      "total_segments_in_traj",
      "current_segment_in_traj"
  };

  // Claim all S32 pins at once
  if (!claim_hardware_resources<machinekit_interfaces::HALS32PinInterface,
                              machinekit_interfaces::HALS32PinHandle>(
          robot_hw, claimed_resources, s32_rsrc_handles, s32_rsrc_names))
      return false;

  // Assign the handles and their pointers
  move_id_handle_ = s32_rsrc_handles[0];
  velocity_scale_manager_->active_move_handler_->setMoveIdHandle(move_id_handle_);
  feedhold_state_handle_ = s32_rsrc_handles[1];
  total_segments_in_traj_handle_ = s32_rsrc_handles[2];
  current_segment_in_traj_handle_ = s32_rsrc_handles[3];

  return true;
}

// WARNING do not early abort from this function, it must clean up probe capture
// mode
template <class SegmentImpl, class HardwareInterface>
void InterruptibleJointTrajectoryController<
    SegmentImpl, HardwareInterface>::update(const ros::Time& time,
                                            const ros::Duration& period)
{
  // Acquire the trajectory pointer from the RT box ONCE (here), and pass to
  // various methods as needed

  ExtendedTrajectoryPtr curr_traj_ptr;
  joint_trajectory_controller::TimeData time_data;

  velocity_scale_manager_->updateVelocityScales(period.toSec());
  double current_scaling_factor =
      velocity_scale_manager_->getCurrentScalingFactor();

  velocity_scale_handle_.set(current_scaling_factor);

  this->velocity_scale_ = current_scaling_factor;

  // Update feedhold state
  bool feedhold_active = velocity_scale_manager_->feedhold_handler_->getFeedholdStatus();

  feedhold_state_handle_.set(feedhold_active ? 1 : 0);

  double target_max_vel_scale_goal =
      velocity_scale_manager_->maxvel_scale_->getTargetScalingFactor();

  std::string scale_factor_name =
      velocity_scale_manager_->maxvel_scale_->SCALE_FACTOR_PARAM_NAME;
  double velocity_scale_limit_on_safety_input = 0.1;

  if (!safety_input_handle_.get() && safety_input_previous_state_ == true)
  {
    scale_factor_before_safety_trip_ = target_max_vel_scale_goal;

    if (scale_factor_before_safety_trip_ > velocity_scale_limit_on_safety_input)
    {
      velocity_scale_manager_->maxvel_scale_->updateTargetScalingFactor(
          velocity_scale_limit_on_safety_input);

      redis_store_msgs::ParamUpdate msg;
      msg.param_name = scale_factor_name;

      std::stringstream stream;
      stream << std::fixed << std::setprecision(2)
             << velocity_scale_limit_on_safety_input;
      msg.param_value = stream.str();
    }
  }
  else if (!safety_input_handle_.get() && safety_input_previous_state_ == false)
  {
    if (target_max_vel_scale_goal != velocity_scale_limit_on_safety_input)
    {
      scale_factor_before_safety_trip_ = target_max_vel_scale_goal;
    }
  }

  else if (safety_input_handle_.get() && safety_input_previous_state_ == false)
  {
    if (scale_factor_before_safety_trip_ > velocity_scale_limit_on_safety_input)
    {
      velocity_scale_manager_->maxvel_scale_->updateTargetScalingFactor(
          scale_factor_before_safety_trip_);

      redis_store_msgs::ParamUpdate msg;
      msg.param_name = scale_factor_name;

      std::stringstream stream;
      stream << std::fixed << std::setprecision(2)
             << scale_factor_before_safety_trip_;
      msg.param_value = stream.str();

      comm_thread_->send(msg);
    }
  }

  safety_input_previous_state_ = safety_input_handle_.get();

  auto period_now = ros::Duration(current_scaling_factor * period.toSec());

  JointTrajectoryControllerType::prepare_for_update(time, period_now,
                                                    curr_traj_ptr, time_data);

  double elapsed_time_within_current_traj = time_data.uptime.toSec();
  elapsed_trajectory_time_handle_.set(elapsed_time_within_current_traj);

  double absolute_time_now = time.toSec();  // Current time, e.g. 1738434042.016589

  // choose different time reference
  // double time_since_jan1_2025 = absolute_time_now - 1735689600.0;  // Time since Jan 1st 2025
  double time_since_feb1_2025 = absolute_time_now - 1738368000.0;  // Time since Feb 1st 2025

  absolute_time_handle_.set(time_since_feb1_2025);

  if (!curr_traj_ptr->started)
  {
    curr_traj_ptr->started = true;
    this->rt_stop_event_triggered_ = false;

    int total_segments = -1;
    for (const auto& joint_traj : curr_traj_ptr->trajectory) {
      total_segments = std::max(total_segments, static_cast<int>(joint_traj.size()));
    }
    total_segments_in_traj_handle_.set(total_segments);
  }

  current_segments_.clear();

  int current_segment_idx = -1;

  // For each joint, find and store the current segment
  for (size_t i = 0; i < this->getNumberOfJoints(); ++i)
  {
    auto& joint_trajectory = curr_traj_ptr->trajectory[i];
    auto segment_it = trajectory_interface::findSegment(
        joint_trajectory, time_data.uptime.toSec());

    if (segment_it != joint_trajectory.end())
    {
      current_segments_.push_back(&(*segment_it));

      // Get the segment index (distance from beginning)
      int idx = std::distance(joint_trajectory.begin(), segment_it);
      current_segment_idx = std::max(current_segment_idx, idx);
    }
  }

  current_segment_in_traj_handle_.set(current_segment_idx);

  int joint_idx = 0;

  // capture quintic trajectory segment coefficients for each joint
  if (!current_segments_.empty() && current_segments_[joint_idx] != nullptr)
  {
    // Use the first position value as an example
    joint1_start_time_handle_.set(current_segments_[joint_idx]->start_time_);
    joint1_duration_handle_.set(current_segments_[joint_idx]->duration_);
    joint1_a_handle_.set(current_segments_[joint_idx]->coefs_[0][5]);
    joint1_b_handle_.set(current_segments_[joint_idx]->coefs_[0][4]);
    joint1_c_handle_.set(current_segments_[joint_idx]->coefs_[0][3]);
    joint1_d_handle_.set(current_segments_[joint_idx]->coefs_[0][2]);
    joint1_e_handle_.set(current_segments_[joint_idx]->coefs_[0][1]);
    joint1_f_handle_.set(current_segments_[joint_idx]->coefs_[0][0]);
  }

  joint_idx = 1;

  if (!current_segments_.empty() && current_segments_[joint_idx] != nullptr)
  {
    // Use the first position value as an example
    joint2_start_time_handle_.set(current_segments_[joint_idx]->start_time_);
    joint2_duration_handle_.set(current_segments_[joint_idx]->duration_);
    joint2_a_handle_.set(current_segments_[joint_idx]->coefs_[0][5]);
    joint2_b_handle_.set(current_segments_[joint_idx]->coefs_[0][4]);
    joint2_c_handle_.set(current_segments_[joint_idx]->coefs_[0][3]);
    joint2_d_handle_.set(current_segments_[joint_idx]->coefs_[0][2]);
    joint2_e_handle_.set(current_segments_[joint_idx]->coefs_[0][1]);
    joint2_f_handle_.set(current_segments_[joint_idx]->coefs_[0][0]);
  }

  joint_idx = 2;

  if (!current_segments_.empty() && current_segments_[joint_idx] != nullptr)
  {
    // Use the first position value as an example
    joint3_start_time_handle_.set(current_segments_[joint_idx]->start_time_);
    joint3_duration_handle_.set(current_segments_[joint_idx]->duration_);
    joint3_a_handle_.set(current_segments_[joint_idx]->coefs_[0][5]);
    joint3_b_handle_.set(current_segments_[joint_idx]->coefs_[0][4]);
    joint3_c_handle_.set(current_segments_[joint_idx]->coefs_[0][3]);
    joint3_d_handle_.set(current_segments_[joint_idx]->coefs_[0][2]);
    joint3_e_handle_.set(current_segments_[joint_idx]->coefs_[0][1]);
    joint3_f_handle_.set(current_segments_[joint_idx]->coefs_[0][0]);
  }

  joint_idx = 3;

  if (!current_segments_.empty() && current_segments_[joint_idx] != nullptr)
  {
    // Use the first position value as an example
    joint4_start_time_handle_.set(current_segments_[joint_idx]->start_time_);
    joint4_duration_handle_.set(current_segments_[joint_idx]->duration_);
    joint4_a_handle_.set(current_segments_[joint_idx]->coefs_[0][5]);
    joint4_b_handle_.set(current_segments_[joint_idx]->coefs_[0][4]);
    joint4_c_handle_.set(current_segments_[joint_idx]->coefs_[0][3]);
    joint4_d_handle_.set(current_segments_[joint_idx]->coefs_[0][2]);
    joint4_e_handle_.set(current_segments_[joint_idx]->coefs_[0][1]);
    joint4_f_handle_.set(current_segments_[joint_idx]->coefs_[0][0]);
  }

  joint_idx = 4;

  if (!current_segments_.empty() && current_segments_[joint_idx] != nullptr)
  {
    // Use the first position value as an example
    joint5_start_time_handle_.set(current_segments_[joint_idx]->start_time_);
    joint5_duration_handle_.set(current_segments_[joint_idx]->duration_);
    joint5_a_handle_.set(current_segments_[joint_idx]->coefs_[0][5]);
    joint5_b_handle_.set(current_segments_[joint_idx]->coefs_[0][4]);
    joint5_c_handle_.set(current_segments_[joint_idx]->coefs_[0][3]);
    joint5_d_handle_.set(current_segments_[joint_idx]->coefs_[0][2]);
    joint5_e_handle_.set(current_segments_[joint_idx]->coefs_[0][1]);
    joint5_f_handle_.set(current_segments_[joint_idx]->coefs_[0][0]);
  }

  joint_idx = 5;

  if (!current_segments_.empty() && current_segments_[joint_idx] != nullptr)
  {
    // Use the first position value as an example
    joint6_start_time_handle_.set(current_segments_[joint_idx]->start_time_);
    joint6_duration_handle_.set(current_segments_[joint_idx]->duration_);
    joint6_a_handle_.set(current_segments_[joint_idx]->coefs_[0][5]);
    joint6_b_handle_.set(current_segments_[joint_idx]->coefs_[0][4]);
    joint6_c_handle_.set(current_segments_[joint_idx]->coefs_[0][3]);
    joint6_d_handle_.set(current_segments_[joint_idx]->coefs_[0][2]);
    joint6_e_handle_.set(current_segments_[joint_idx]->coefs_[0][1]);
    joint6_f_handle_.set(current_segments_[joint_idx]->coefs_[0][0]);
  }

  typename JointTrajectoryControllerType::RealtimeGoalHandlePtr
      current_active_goal(this->rt_active_goal_);

  // React to estop and stop events

  handle_estop_event(curr_traj_ptr, time_data, current_active_goal);
  handle_stop_event(curr_traj_ptr, time_data, current_active_goal);

  // React to probe transitions
  handle_probe_transitions(curr_traj_ptr, time_data, current_active_goal);

  const double const_current_scaling_factor = current_scaling_factor;

  JointTrajectoryControllerType::update_joint_trajectory(
      curr_traj_ptr->trajectory, time_data, period_now, const_current_scaling_factor);
}

template <class SegmentImpl, class HardwareInterface>
void InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
    handle_estop_event(
        ExtendedTrajectoryPtr curr_traj_ptr,
        joint_trajectory_controller::TimeData time_data,
        typename JointTrajectoryControllerType::RealtimeGoalHandlePtr
            current_active_goal)
{
  if (!estop_handle_.get())
    return;  // No event

  error_code_.set(GetJointTrajectoryErrorContextResponse::HARDWARE_ESTOP_EVENT);
  this->abortActiveGoalWithError(
      current_active_goal, time_data.uptime,
      GetJointTrajectoryErrorContextResponse::HARDWARE_ESTOP_EVENT);
}

template <class SegmentImpl, class HardwareInterface>
void InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
    handle_stop_event(
        ExtendedTrajectoryPtr curr_traj_ptr,
        joint_trajectory_controller::TimeData time_data,
        typename JointTrajectoryControllerType::RealtimeGoalHandlePtr
            current_active_goal)
{
  if (!stop_handle_.get())
    return;  // No event

  this->setHoldPosition(time_data.uptime, current_active_goal);

  error_code_.set(GetJointTrajectoryErrorContextResponse::HARDWARE_STOP_EVENT);
  this->cancelActiveGoalWithError(
      current_active_goal, time_data.uptime,
      GetJointTrajectoryErrorContextResponse::HARDWARE_STOP_EVENT);
  // This is a request to the controller; mark request completed
  stop_handle_.set(0);
}


template <class SegmentImpl, class HardwareInterface>
void InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
    handle_probe_transitions(
        ExtendedTrajectoryPtr curr_traj_ptr,
        joint_trajectory_controller::TimeData time_data,
        typename JointTrajectoryControllerType::RealtimeGoalHandlePtr
            current_active_goal)
{
  // First, ensure that a new trajectory's probe settings are applied
  ProbeSettings& settings = curr_traj_ptr->motion_settings;
  if (!curr_traj_ptr->started)
  {
    // Disregard any previously captured probe transitions (up to and including
    // the current timestep) This means that probe transitions are not detected
    // on the very first timestep
    error_code_.set(GetJointTrajectoryErrorContextResponse::SUCCESSFUL);
    probe_handle_.startNewProbeCapture(settings.probe_request_capture_type);
    probe_handle_.acquireProbeTransition();
    // Don't re-apply the settings now that the new trajectory is active
    curr_traj_ptr->started = true;
    // Since we're starting a new trajectory, the previous "stop" event is over
    this->rt_stop_event_triggered_ = false;
  }

  auto probe_transition = probe_handle_.acquireProbeTransition();
  auto const probe_capture_type = probe_handle_.getProbeCapture();

  switch (probe_transition)
  {
    case ProbeTransitions::RISING:
      switch (probe_capture_type)
      {
        case stop_event_msgs::SetNextProbeMoveRequest::
            PROBE_REQUIRE_RISING_EDGE:
        case stop_event_msgs::SetNextProbeMoveRequest::
            PROBE_OPTIONAL_RISING_EDGE:
          this->completeActiveGoal(current_active_goal, time_data.uptime);
          break;
        case stop_event_msgs::SetNextProbeMoveRequest::PROBE_IGNORE_INPUT:
          break;
        default:
          // "RETRACT" is meant to retract off of a surface and continue moving,
          // but should stop if it hits something else
          this->abortActiveGoalWithError(
              current_active_goal, time_data.uptime,
              GetJointTrajectoryErrorContextResponse::
                  PROBE_UNEXPECTED_RISING_EDGE);
          break;
      }
      break;
    case ProbeTransitions::FALLING:
      switch (probe_capture_type)
      {
        case stop_event_msgs::SetNextProbeMoveRequest::
            PROBE_REQUIRE_FALLING_EDGE:
        case stop_event_msgs::SetNextProbeMoveRequest::
            PROBE_OPTIONAL_FALLING_EDGE:
          this->completeActiveGoal(current_active_goal, time_data.uptime);
          break;
        case stop_event_msgs::SetNextProbeMoveRequest::PROBE_RETRACT:
        case stop_event_msgs::SetNextProbeMoveRequest::PROBE_IGNORE_INPUT:
          break;
        default:
          this->abortActiveGoalWithError(
              current_active_goal, time_data.uptime,
              GetJointTrajectoryErrorContextResponse::
                  PROBE_UNEXPECTED_FALLING_EDGE);
          break;
      }
      break;
    case ProbeTransitions::NONE:
      // Probe should not be high for non-probe motions, or if we're looking for
      // a rising edge (and haven't found it yet)
      if (probe_handle_.getProbeState())
      {
        switch (probe_capture_type)
        {
          case stop_event_msgs::SetNextProbeMoveRequest::
              PROBE_REQUIRE_FALLING_EDGE:
          case stop_event_msgs::SetNextProbeMoveRequest::
              PROBE_OPTIONAL_FALLING_EDGE:
          case stop_event_msgs::SetNextProbeMoveRequest::PROBE_RETRACT:
          case stop_event_msgs::SetNextProbeMoveRequest::PROBE_IGNORE_INPUT:
            // We're ok, expect the probe to be active for these motion types
            break;
          case stop_event_msgs::SetNextProbeMoveRequest::
              PROBE_REQUIRE_RISING_EDGE:
          case stop_event_msgs::SetNextProbeMoveRequest::
              PROBE_OPTIONAL_RISING_EDGE:
          default:  // Deliberate fallthrough
            this->abortActiveGoalWithError(
                current_active_goal, time_data.uptime,
                GetJointTrajectoryErrorContextResponse::PROBE_CONTACT_AT_START);
            break;
        }
      }
      break;
    default:
      this->abortActiveGoalWithError(
          current_active_goal, time_data.uptime,
          GetJointTrajectoryErrorContextResponse::PROBE_INVALID_STATE);
      break;
  }
}

template <class SegmentImpl, class HardwareInterface>
void InterruptibleJointTrajectoryController<
    SegmentImpl, HardwareInterface>::onTrajectoryError(int error_code)
{
  // Publish the error code
  error_code_.set(error_code);
}

/**
 * Check if all joints have reached their goal state, and mark the goal handle
 * as succeeded if so. Derived classes can specialize this if they need finer
 * control over goal success (e.g. if there are additional criteria like for
 * probing).
 */
template <class SegmentImpl, class HardwareInterface>
void InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
    checkReachedTrajectoryGoal(const ros::Time& uptime)
{
  int capture_type = probe_handle_.getProbeCapture();
  if (capture_type)
  {
    checkReachedTrajectoryGoalProbe(capture_type, uptime);
  }
  else
  {
    // Normal moves get forwarded to the stock goal check
    JointTrajectoryControllerType::checkReachedTrajectoryGoal(uptime);
  }
}

template <class SegmentImpl, class HardwareInterface>
void InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
    checkReachedTrajectoryGoalProbe(int capture_type, const ros::Time& uptime)
{
  // Check if we have reached the end of a
  if (this->successful_joint_traj_.count() != this->getNumberOfJoints())
    return;  // We hav
  RealtimeGoalHandlePtr current_active_goal(this->rt_active_goal_);
  if (!current_active_goal)
    return;  // en't

  if (capture_type ==
          stop_event_msgs::SetNextProbeMoveRequest::PROBE_REQUIRE_RISING_EDGE ||
      capture_type ==
          stop_event_msgs::SetNextProbeMoveRequest::PROBE_REQUIRE_FALLING_EDGE)
  {
    this->abortActiveGoalWithError(
        current_active_goal, uptime,
        GetJointTrajectoryErrorContextResponse::PROBE_REACHED_MOTION_END);
  }
  else
  {
    this->completeActiveGoal(current_active_goal, uptime);
  }
  this->rt_active_goal_.reset();
  // TODO pass uptime in here to plane a stop trajectory in case the goal has
  // nonzero velocity?
  this->successful_joint_traj_.reset();
}

template <class SegmentImpl, class HardwareInterface>
bool InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
    handleProbeRequest(stop_event_msgs::SetNextProbeMoveRequest& request,
                       stop_event_msgs::SetNextProbeMoveResponse& response)
{
  ROS_INFO_STREAM("Probe capture requested, mode " << request.mode);
  this->queued_motion_settings_.probe_request_capture_type =
      (machinekit_interfaces::ProbeCaptureType)request.mode;
  return true;
}

template <class SegmentImpl, class HardwareInterface>
bool InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
    handleStopEventResultRequest(
        stop_event_msgs::GetStopEventResultRequest& request,
        stop_event_msgs::GetStopEventResultResponse& response)
{
  response.stop_event = (long)probe_handle_.getProbeResultType();
  response.event_time = probe_handle_.getProbeCaptureTime();
  ROS_INFO_STREAM("Handling request for probe results for capture type "
                  << response.stop_event << " at time " << response.event_time);
  if (response.stop_event)
  {
    std::vector<double> event_positions(this->getNumberOfJoints(), 0.0);
    for (unsigned int joint_index = 0; joint_index < this->getNumberOfJoints();
         ++joint_index)
    {
      // Hope that the lock thrashing here doesn't affect RT...
      // response.result.joint_names[joint_index] =
      // this->joint_names_[joint_index];
      event_positions[joint_index] =
          probe_joint_results_[joint_index].getPosition();
      // probe_state.velocities[joint_index] =
      // probe_joint_results_[joint_index].getVelocity();
      // probe_state.positions[joint_index] = 0;
      // probe_state.velocities[joint_index] = 0;
    }
    response.event_position = event_positions;
  }
  return true;
}

template <class SegmentImpl, class HardwareInterface>
bool InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
    handleJointTrajectoryErrorContextRequest(
        stop_event_msgs::GetJointTrajectoryErrorContextRequest& request,
        stop_event_msgs::GetJointTrajectoryErrorContextResponse& response)
{
  response.error_code = error_code_.get();
  ROS_INFO_STREAM("Handling request for error details (current error code is "
                  << response.error_code << ")");
  return true;
}

template <class SegmentImpl, class HardwareInterface>
bool InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
    updateTrajectoryCommand(const JointTrajectoryConstPtr& msg,
                            RealtimeGoalHandlePtr gh, std::string* error_string)
{
  // Guard against starting a motion if the probe is currently active
  // KLUDGE this should really be done in realtime but this will have to do
  if (probe_handle_.getProbeState() > 0)
  {
    auto requested_capture =
        this->queued_motion_settings_.probe_request_capture_type;
    switch (requested_capture)
    {
      case SetNextProbeMoveRequest::PROBE_NONE:
      case SetNextProbeMoveRequest::PROBE_OPTIONAL_RISING_EDGE:
      case SetNextProbeMoveRequest::PROBE_REQUIRE_RISING_EDGE: {
        if (error_string)
        {
          const std::string err_msg("Can't start a motion or probe move with "
                                    "probe active in probing mode " +
                                    std::to_string(requested_capture));
          *error_string = err_msg;
        }
        else
        {
          // Caller didn't provide a feedback mechanism, so complain directly to
          // the console
          (++jog_err_count_) %= jog_err_threshold_;
          if (!jog_err_count_)
          {
            // KLUDGE avoid console spam by only publishing once we reach the
            // threshold (since these errors usually come from continuous
            // jogging, which tends to be a stream of updates rather than
            // one-off commands)
            ROS_ERROR_STREAM("Can't start a jog motion with probe active. "
                             "Verify probe connection / polarity, click 'Jog "
                             "Ignore Probe' to re-enable jogging, then "
                             "carefully jog the probe to a safe position.");
          }
        }
        this->claimQueuedSettings();
        error_code_.set(
            GetJointTrajectoryErrorContextResponse::PROBE_CONTACT_AT_START);
        return false;
      }
      default:
        break;
    }
  }
  // NOTE: this clears the queued settings once they're accepted
  bool res = JointTrajectoryControllerType::updateTrajectoryCommand(
      msg, gh, error_string);
  if (res)
  {
    // Sends a message immediately when the next jog error occurs, but then
    // silences any additional errors until the threshold is reached.
    jog_err_count_ = -1;
  }
  return res;
}


}  // namespace interruptible_joint_trajectory_controller
