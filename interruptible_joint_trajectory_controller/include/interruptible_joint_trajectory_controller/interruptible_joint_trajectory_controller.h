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

// ros_controls
#include <realtime_tools/realtime_server_goal_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/internal/demangle_symbol.h>

#include <machinekit_interfaces/realtime_event_interface.h>
#include <machinekit_interfaces/probe_interface.h>
#include <machinekit_interfaces/joint_event_interface.h>

// Bring in enums
using machinekit_interfaces::ProbeTransitions;
using machinekit_interfaces::ProbeState;
using stop_event_msgs::SetNextProbeMoveRequest;
using stop_event_msgs::SetNextProbeMoveResponse;

// Project
#include <joint_trajectory_controller/joint_trajectory_controller.h>

namespace interruptible_joint_trajectory_controller
{
static const std::string PROBE_SERVICE_NAME{ "probe" };
static const std::string PROBE_RESULT_SERVICE_NAME{ "probe_result" };

/**
 * Controller for executing joint-space trajectories on a group of joints, that can respond to stop events triggered from hardware in the realtime loop.
 *
 * See JointTrajectoryController documentation for details on how the trajectory execution works.
 *
 */
template <class SegmentImpl, class HardwareInterface>
class InterruptibleJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>
{
protected:
    using JointTrajectoryControllerType = typename joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>;
    using typename JointTrajectoryControllerType::JointTrajectoryConstPtr;
    using typename JointTrajectoryControllerType::RealtimeGoalHandlePtr;
    using typename JointTrajectoryControllerType::Trajectory;

public:

    InterruptibleJointTrajectoryController();

    /** \name Non Real-Time Safe Functions
   *\{*/

    // KLUDGE have to override this method in Controller to be able to initialize multiple hardware interface types
    bool initRequest(hardware_interface::RobotHW* robot_hw,
                     ros::NodeHandle&             root_nh,
                     ros::NodeHandle&             controller_nh,
                     controller_interface::ControllerBase::ClaimedResources&            claimed_resources) override;
    /*\}*/

    /** \name Real-Time Safe Functions
   *\{*/

    void abortActiveGoalWithError(const ros::Time& time, int error_code, std::string const &&explanation); // Like preemptActiveGoal but marks it as failed
    void completeActiveGoal(const ros::Time &time); // Like preemptActiveGoal but for when an external goal state is reached (i.e. probing)

    void update(const ros::Time& time, const ros::Duration& period);
    /*\}*/

    // Command handling, not real-time safe
    virtual bool updateTrajectoryCommand(const JointTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh, std::string* error_string = nullptr);

    // Service calls, not real-time safe

    /**
     * @brief Service callback to force the controller into the hold position.
     *
     * @param request Dummy for triggering the service
     * @param response True on success.
     *
     * @return False if something went wrong. True otherwise.
     */
    bool handleProbeRequest(stop_event_msgs::SetNextProbeMoveRequest& request, stop_event_msgs::SetNextProbeMoveResponse& response);
    bool handleStopEventResultRequest(stop_event_msgs::GetStopEventResultRequest& request, stop_event_msgs::GetStopEventResultResponse& response);

protected:
    virtual void checkReachedTrajectoryGoal();
    virtual void checkReachedTrajectoryGoalProbe(int capture_type);
    // Services for controller trajectory behavior
    ros::ServiceServer probe_service_; //!< Declare success when probe trip occurs on the next send trajectory
    ros::ServiceServer probe_result_service_; //!< Request the result of a probe

    std::vector<machinekit_interfaces::JointEventDataHandle> probe_joint_results_;
    machinekit_interfaces::ProbeHandle probe_handle;
    machinekit_interfaces::RealtimeEventHandle stop_event;
    bool stop_event_triggered_;
};

} // namespace

#pragma once

namespace interruptible_joint_trajectory_controller
{

template <class SegmentImpl, class HardwareInterface>
InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
InterruptibleJointTrajectoryController()
    : JointTrajectoryControllerType()
{
}


template<class SegmentImpl, class HardwareInterface>
bool InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::initRequest(
        hardware_interface::RobotHW *robot_hw,
        ros::NodeHandle &root_nh,
        ros::NodeHandle &controller_nh,
        controller_interface::ControllerBase::ClaimedResources &claimed_resources)
{
    // Initialize auxiliary hardware interfaces required by this controller (probe / stop event interfaces)
    ROS_INFO_STREAM(
                "initRequest for InterruptibleJointTrajectoryController");

    // SO ugly, need to redirect to the base class method, but this is fragile if JointTrajectoryController ever decides to add one...
    // Complete the underlying initialization for the controller (JointTrajectoryController base-level init)
    bool base_init = JointTrajectoryControllerType::initRequest(robot_hw, root_nh, controller_nh, claimed_resources);
    if (!base_init) {
        return false;
    }

    // This is following the same basic pattern as Controller<>'s initRequest, but we can't just use the basic init function because it's tied to the Joint interface.
    // initRequest itself has to be overridden to handle these auxiliary interfaces.
    {
        ROS_INFO_STREAM_NAMED(this->name_, "Claiming probe resources");
        machinekit_interfaces::ProbeInterface* probe_intf = robot_hw->get<machinekit_interfaces::ProbeInterface>();
        auto hw_if_typename = hardware_interface::internal::demangledTypeName<machinekit_interfaces::ProbeInterface>();
        if (!probe_intf)
        {
            ROS_ERROR("This controller requires a hardware interface of type '%s'."
                                " Make sure this is registered in the hardware_interface::RobotHW class.",
                                hw_if_typename.c_str());
            return false;
        }
        // return which resources are claimed by this controller
        probe_intf->clearClaims();

        try {
            probe_handle = probe_intf->getHandle("probe");
        }
        catch (...)
        {
            ROS_ERROR_STREAM_NAMED(this->name_, "Could not find probe in '" <<
                                                         hw_if_typename << "'.");
            return false;
        }
        hardware_interface::InterfaceResources iface_res(hw_if_typename, probe_intf->getClaims());
        claimed_resources.push_back(iface_res);
        probe_intf->clearClaims();
    }
    {
        ROS_INFO_STREAM_NAMED(this->name_, "Claiming joint event data resources");
        machinekit_interfaces::JointEventDataInterface* probe_data_intf = robot_hw->get<machinekit_interfaces::JointEventDataInterface>();
        auto hw_if_typename = hardware_interface::internal::demangledTypeName<machinekit_interfaces::JointEventDataInterface>();
        if (!probe_data_intf)
        {
            ROS_ERROR("This controller requires a hardware interface of type '%s'."
                                " Make sure this is registered in the hardware_interface::RobotHW class.",
                                hw_if_typename.c_str());
            return false;
        }
        // return which resources are claimed by this controller
        probe_data_intf->clearClaims();

        // NOTE: this depends on successful initialization of the controller so we can reuse the found joint names to create probe results per joint
        const unsigned int n_joints = this->joint_names_.size();
        probe_joint_results_.resize(n_joints);
        for (unsigned int i = 0; i < n_joints; ++i)
        {
            std::string const &jname = this->joint_names_[i]+"_probe";
            // Uses a parallel set of handles defined by the joint names (to avoid conflicts with the standard joint handles)
            try {probe_joint_results_[i] = probe_data_intf->getHandle(jname);}
            catch (...)
            {
                ROS_ERROR_STREAM_NAMED(this->name_, "Could not find joint '" << jname << "' in '" <<
                                                             this->getHardwareInterfaceType() << "'.");
                return false;
            }
        }

        hardware_interface::InterfaceResources iface_res(hw_if_typename, probe_data_intf->getClaims());
        claimed_resources.push_back(iface_res);
        probe_data_intf->clearClaims();
    }

    ROS_INFO_STREAM_NAMED(this->name_, "Claimed " << claimed_resources.size() << " interfaces");
    for (auto const &s : claimed_resources) {
        for (auto const &c : s.resources) {
            ROS_INFO_STREAM_NAMED(this->name_, "interface " << s.hardware_interface << " claims " << c);
    }}
    ROS_INFO_STREAM_NAMED(this->name_, "Starting probe services");
    // Set up services to control probe behavior
    probe_service_ = controller_nh.advertiseService(PROBE_SERVICE_NAME, &InterruptibleJointTrajectoryController::handleProbeRequest, this);
    probe_result_service_ = controller_nh.advertiseService(PROBE_RESULT_SERVICE_NAME, &InterruptibleJointTrajectoryController::handleStopEventResultRequest, this);
    // success
    this->state_ = controller_interface::Controller<HardwareInterface>::ControllerState::INITIALIZED;
    return true;
}


template <class SegmentImpl, class HardwareInterface>
inline void InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
abortActiveGoalWithError(const ros::Time& time, int error_code, std::string const &&explanation)
{
    typename JointTrajectoryControllerType::RealtimeGoalHandlePtr current_active_goal(this->rt_active_goal_);

    // Cancels the currently active goal
    if (current_active_goal && !stop_event_triggered_)
    {
        // TODO standardize error codes
        current_active_goal->preallocated_result_->error_code = -6;
        // TODO confirm realtime safety of this assignment (e.g. is the string reserved, or does this trigger an allocation?)
        current_active_goal->preallocated_result_->error_string = explanation;
        current_active_goal->setAborted(current_active_goal->preallocated_result_);
        // FIXME this fails noisily if the stop trajectory duration is 0.0
        this->setHoldPosition(time, current_active_goal);
        stop_event_triggered_ = true;
    }
}

template <class SegmentImpl, class HardwareInterface>
inline void InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
completeActiveGoal(const ros::Time &time)
{
    typename JointTrajectoryControllerType::RealtimeGoalHandlePtr current_active_goal(this->rt_active_goal_);

    // Cancels the currently active goal
    if (current_active_goal && !stop_event_triggered_)
    {
        // Marks the current goal as canceled
        current_active_goal->preallocated_result_->error_code = 0;
        current_active_goal->setSucceeded(current_active_goal->preallocated_result_);
        this->setHoldPosition(time, current_active_goal);
        stop_event_triggered_ = true;
    }
}

// WARNING do not early abort from this function, it must clean up probe capture mode
template <class SegmentImpl, class HardwareInterface>
void InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
update(const ros::Time& time, const ros::Duration& period)
{
    RealtimeGoalHandlePtr current_active_goal(this->rt_active_goal_);

    auto probe_transition = (ProbeTransitions)probe_handle.acquireProbeTransition();

    // FIXME does this need to be incremented here?
    auto const time_data_tmp = *(this->time_data_.readFromRT());
    auto const uptime = time_data_tmp.uptime + period;
    switch (probe_transition) {
    case ProbeTransitions::RISING:
        switch (probe_handle.getProbeCapture()) {
        case (int)stop_event_msgs::SetNextProbeMoveRequest::PROBE_REQUIRE_RISING_EDGE:
        case (int)stop_event_msgs::SetNextProbeMoveRequest::PROBE_OPTIONAL_RISING_EDGE:
            completeActiveGoal(uptime);
            break;
        case (int)stop_event_msgs::SetNextProbeMoveRequest::PROBE_IGNORE_INPUT:
            break;
        default:
            abortActiveGoalWithError(uptime, -6, "Unexpected probe rising edge during probe motion");
            break;
        }
        break;
    case ProbeTransitions::FALLING:
        switch (probe_handle.getProbeCapture()) {
        case (int)stop_event_msgs::SetNextProbeMoveRequest::PROBE_REQUIRE_FALLING_EDGE:
        case (int)stop_event_msgs::SetNextProbeMoveRequest::PROBE_OPTIONAL_FALLING_EDGE:
            completeActiveGoal(uptime);
            break;
        case (int)stop_event_msgs::SetNextProbeMoveRequest::PROBE_IGNORE_INPUT:
            break;
        default:
            abortActiveGoalWithError(uptime, -7, "Unexpected probe falling edge during motion");
            break;
        }
        break;
    default:
        if (probe_handle.getProbeState() > 0 && probe_handle.getProbeCapture() < 1) {
            abortActiveGoalWithError(uptime, -8, "Unexpected probe signal during non-probe motion");
            break;
        }
    }

    // TODO mechanism to throw clear error if probe move reaches end. For example, zero out goal tolerances? This may need a custom update function to get a real error message.
    JointTrajectoryControllerType::update(time, period);

    // Reset probe capture if goal is done (regardless of success / failure)
    if (!this->rt_active_goal_ && current_active_goal) {
        probe_handle.setProbeCapture(0);
    }
}

/**
 * Check if all joints have reached their goal state, and mark the goal handle as succeeded if so.
 * Derived classes can specialize this if they need finer control over goal success
 * (e.g. if there are additional criteria like for probing).
 */
template <class SegmentImpl, class HardwareInterface>
void InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
checkReachedTrajectoryGoal()
{
    int capture_type = probe_handle.getProbeCapture();
    if (capture_type) {
        checkReachedTrajectoryGoalProbe(capture_type);
    } else {
        // Normal moves get forwarded to the stock goal check
        JointTrajectoryControllerType::checkReachedTrajectoryGoal();
    }
}

template <class SegmentImpl, class HardwareInterface>
void InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
checkReachedTrajectoryGoalProbe(int capture_type)
{
    // Check if we have reached the end of a
    RealtimeGoalHandlePtr current_active_goal(this->rt_active_goal_);
    if (current_active_goal && this->successful_joint_traj_.count() == this->getNumberOfJoints())
    {
        if (!stop_event_triggered_ && (capture_type == stop_event_msgs::SetNextProbeMoveRequest::PROBE_REQUIRE_RISING_EDGE ||
             capture_type == stop_event_msgs::SetNextProbeMoveRequest::PROBE_REQUIRE_FALLING_EDGE)) {
          current_active_goal->preallocated_result_->error_code = -9;
          current_active_goal->preallocated_result_->error_string = "Reached end of probe motion without probe trip";
          current_active_goal->setAborted(current_active_goal->preallocated_result_);
        } else {
            // Declare success because we reached the end of the motion (or we've successfully stopped after a probe trip)
            current_active_goal->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
            current_active_goal->setSucceeded(current_active_goal->preallocated_result_);
            current_active_goal.reset(); // do not publish feedback
        }
        this->rt_active_goal_.reset();
        // TODO pass uptime in here to plane a stop trajectory in case the goal has nonzero velocity?
        this->successful_joint_traj_.reset();
    }
}

template <class SegmentImpl, class HardwareInterface>
bool InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::handleProbeRequest(stop_event_msgs::SetNextProbeMoveRequest &request, stop_event_msgs::SetNextProbeMoveResponse &response)
{
    ROS_INFO_STREAM("Probe capture requested, mode " << request.mode);
    probe_handle.setProbeCapture(request.mode);
    return true;
}

template <class SegmentImpl, class HardwareInterface>
bool InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::handleStopEventResultRequest(stop_event_msgs::GetStopEventResultRequest &request, stop_event_msgs::GetStopEventResultResponse &response)
{
    response.stop_event = probe_handle.getProbeResultType();
    response.event_time = probe_handle.getProbeCaptureTime();
    ROS_INFO_STREAM("Handling request for probe results, stop event " << response.stop_event << ", event_time " << response.event_time);
    if (response.stop_event) {
      // TODO handshake to ensure that probe result is valid and matches the request (so we're not returning old data?)
      std::vector<double> probe_positions(this->getNumberOfJoints(), 0.0);
      for (unsigned int joint_index = 0; joint_index < this->getNumberOfJoints(); ++joint_index)
      {
        probe_positions[joint_index] = 0;
        // Hope that the lock thrashing here doesn't affect RT...
        //response.result.joint_names[joint_index] = this->joint_names_[joint_index];
        probe_positions[joint_index] = probe_joint_results_[joint_index].getPosition();
        //probe_state.velocities[joint_index] = probe_joint_results_[joint_index].getVelocity();
        //probe_state.positions[joint_index] = 0;
        //probe_state.velocities[joint_index] = 0;
      }
      response.event_position = probe_positions;
    }
    return true;
}

template <class SegmentImpl, class HardwareInterface>
bool InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
updateTrajectoryCommand(const JointTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh, std::string* error_string)
{
    // Guard against starting a motion if the probe is currently active
    // KLUDGE this should really be done in realtime but this will have to do
    if (probe_handle.getProbeState() > 0) {
        switch (probe_handle.getProbeCapture()) {
        case SetNextProbeMoveRequest::PROBE_NONE:
        case SetNextProbeMoveRequest::PROBE_OPTIONAL_RISING_EDGE:
        case SetNextProbeMoveRequest::PROBE_REQUIRE_RISING_EDGE:
        {
          const std::string err_msg("Can't start a motion or probe move with probe active in mode" + std::to_string(probe_handle.getProbeCapture()));
          if (error_string) {
            *error_string = err_msg;
          }
          return false;
        }
        default:
           break;
        }
    }
    // FIXME not RT safe, this is a huge bandaid right now
    stop_event_triggered_ = false;
    return JointTrajectoryControllerType::updateTrajectoryCommand(msg, gh, error_string);
}


} // namespace
