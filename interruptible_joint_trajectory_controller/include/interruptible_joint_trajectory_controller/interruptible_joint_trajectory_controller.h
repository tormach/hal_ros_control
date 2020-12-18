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
#include <stop_event_msgs/StopEventResult.h>
#include <stop_event_msgs/GetProbeResult.h>

// ros_controls
#include <realtime_tools/realtime_server_goal_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/internal/demangle_symbol.h>

#include <machinekit_interfaces/realtime_event_interface.h>
#include <machinekit_interfaces/probe_interface.h>

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
    bool init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    /*\}*/

    // KLUDGE have to override this method in Controller to be able to initialize multiple hardware interface types
    bool initRequest(hardware_interface::RobotHW* robot_hw,
                     ros::NodeHandle&             root_nh,
                     ros::NodeHandle&             controller_nh,
                     controller_interface::ControllerBase::ClaimedResources&            claimed_resources) override;

    /** \name Real-Time Safe Functions
   *\{*/

    void abortActiveGoalWithError(const ros::Time& time, std::string const &&explanation); // Like preemptActiveGoal but marks it as failed
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
    bool handleProbeResultRequest(stop_event_msgs::GetProbeResultRequest& request, stop_event_msgs::GetProbeResultResponse& response);

protected:
    virtual void checkReachedTrajectoryGoal();
    virtual void checkReachedTrajectoryGoalProbe();
    // Services for controller trajectory behavior
    ros::ServiceServer probe_service_; //!< Declare success when probe trip occurs on the next send trajectory
    ros::ServiceServer probe_result_service_; //!< Request the result of a probe

    std::vector<typename JointTrajectoryControllerType::JointHandle> probe_joint_results_;
    machinekit_interfaces::ProbeHandle probe_handle;
    machinekit_interfaces::RealtimeEventHandle stop_event;
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


template <class SegmentImpl, class HardwareInterface>
bool InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::init(HardwareInterface* hw,
                                                                                  ros::NodeHandle&   root_nh,
                                                                                  ros::NodeHandle&   controller_nh)
{
    bool res = JointTrajectoryControllerType::init(hw, root_nh, controller_nh);
    if (!res) {
        return res;
    }

    // NOTE: this depends on successful initialization of the controller so we can reuse the found joint names to create probe results per joint
    const unsigned int n_joints = this->joint_names_.size();
    probe_joint_results_.resize(n_joints);
    for (unsigned int i = 0; i < n_joints; ++i)
    {
        std::string probe_joint_name = "probe_" + this->joint_names_[i];
        try {probe_joint_results_[i] = hw->getHandle(probe_joint_name);}
        catch (...)
        {
            ROS_ERROR_STREAM_NAMED(this->name_, "Could not find joint '" << probe_joint_name << "' in '" <<
                                   this->getHardwareInterfaceType() << "'.");
            return false;
        }
    }

    // Set up services to control probe behavior
    probe_service_ = controller_nh.advertiseService(PROBE_SERVICE_NAME, &InterruptibleJointTrajectoryController::handleProbeRequest, this);
    probe_result_service_ = controller_nh.advertiseService(PROBE_RESULT_SERVICE_NAME, &InterruptibleJointTrajectoryController::handleProbeResultRequest, this);

    return true;
}

template<class SegmentImpl, class HardwareInterface>
bool InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::initRequest(
        hardware_interface::RobotHW *robot_hw,
        ros::NodeHandle &root_nh,
        ros::NodeHandle &controller_nh,
        controller_interface::ControllerBase::ClaimedResources &claimed_resources)
{
    // Initialize auxiliary hardware interfaces required by this controller (probe / stop event interfaces)

    // This is following the same basic pattern as Controller<>'s initRequest, but we can't just use the basic init function because it's tied to the Joint interface.
    // initRequest itself has to be overridden to handle these auxiliary interfaces.
    {
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
        claimed_resources.assign(1, iface_res);
        probe_intf->clearClaims();
    }
    {
        machinekit_interfaces::RealtimeEventInterface* rt_event_intf = robot_hw->get<machinekit_interfaces::RealtimeEventInterface>();
        auto hw_if_typename = hardware_interface::internal::demangledTypeName<machinekit_interfaces::RealtimeEventInterface>();
        if (!rt_event_intf)
        {
            ROS_ERROR("This controller requires a hardware interface of type '%s'."
                      " Make sure this is registered in the hardware_interface::RobotHW class.",
                      hw_if_typename.c_str());
            return false;
        }
        // return which resources are claimed by this controller
        rt_event_intf->clearClaims();

        try {
            stop_event = rt_event_intf->getHandle("probe");
        }
        catch (...)
        {
            ROS_ERROR_STREAM_NAMED(this->name_, "Could not find stop_event in '" <<
                                   hw_if_typename << "'.");
            return false;
        }
        hardware_interface::InterfaceResources iface_res(hw_if_typename, rt_event_intf->getClaims());
        claimed_resources.assign(1, iface_res);
        rt_event_intf->clearClaims();
    }

    // SO ugly, need to redirect to the base class method, but this is fragile if JointTrajectoryController ever decides to add one...
    // Complete the underlying initialization for the controller (probe joint stuff, JointTrajectoryController base-level init)
    return controller_interface::Controller<HardwareInterface>::initRequest(robot_hw, root_nh, controller_nh, claimed_resources);
}


template <class SegmentImpl, class HardwareInterface>
inline void InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
abortActiveGoalWithError(const ros::Time& time, std::string const &&explanation)
{
    typename JointTrajectoryControllerType::RealtimeGoalHandlePtr current_active_goal(this->rt_active_goal_);

    // Cancels the currently active goal
    if (current_active_goal)
    {
        // Marks the current goal as canceled
        this->rt_active_goal_.reset();
        // TODO standardize error codes
        current_active_goal->preallocated_result_->error_code = -6;
        // TODO confirm realtime safety of this assignment (e.g. is the string reserved, or does this trigger an allocation?)
        current_active_goal->preallocated_result_->error_string = explanation;
        current_active_goal->setAborted(current_active_goal->preallocated_result_);
        JointTrajectoryControllerType::setHoldPosition(time);
    }
}

template <class SegmentImpl, class HardwareInterface>
inline void InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
completeActiveGoal(const ros::Time &time)
{
    typename JointTrajectoryControllerType::RealtimeGoalHandlePtr current_active_goal(this->rt_active_goal_);

    // Cancels the currently active goal
    if (current_active_goal)
    {
        // Marks the current goal as canceled
        this->rt_active_goal_.reset();
        // TODO pass details back
        current_active_goal->preallocated_result_->error_code = 0;
        current_active_goal->setSucceeded(current_active_goal->preallocated_result_);
        JointTrajectoryControllerType::setHoldPosition(time);
    }
}

// WARNING do not early abort from this function, it must clean up probe capture mode
template <class SegmentImpl, class HardwareInterface>
void InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
update(const ros::Time& time, const ros::Duration& period)
{
    RealtimeGoalHandlePtr current_active_goal(this->rt_active_goal_);

    auto probe_transition = (ProbeTransitions)probe_handle.acquireProbeTransition();

    switch (probe_transition) {
    case ProbeTransitions::RISING:
        switch (probe_handle.getProbeCapture()) {
        case (int)stop_event_msgs::SetNextProbeMoveRequest::PROBE_REQUIRE_RISING_EDGE:
        case (int)stop_event_msgs::SetNextProbeMoveRequest::PROBE_OPTIONAL_RISING_EDGE:
            completeActiveGoal(time);
            break;
        case (int)stop_event_msgs::SetNextProbeMoveRequest::PROBE_IGNORE_INPUT:
            break;
        default:
            abortActiveGoalWithError(time, "Unexpected probe rising edge during probe motion");
            break;
        }
        break;
    case ProbeTransitions::FALLING:
        switch (probe_handle.getProbeCapture()) {
        case (int)stop_event_msgs::SetNextProbeMoveRequest::PROBE_REQUIRE_FALLING_EDGE:
        case (int)stop_event_msgs::SetNextProbeMoveRequest::PROBE_OPTIONAL_FALLING_EDGE:
            completeActiveGoal(time);
            break;
        case (int)stop_event_msgs::SetNextProbeMoveRequest::PROBE_IGNORE_INPUT:
            break;
        default:
            abortActiveGoalWithError(time, "Unexpected probe falling edge during motion");
            break;
        }
        break;
    default:
        if (probe_handle.getProbeState() > 0 && probe_handle.getProbeCapture() < 1) {
            abortActiveGoalWithError(time, "Unexpected probe signal during non-probe motion");
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
    switch((machinekit_interfaces::ProbeTransitions)probe_handle.getProbeCapture()) {
    case machinekit_interfaces::ProbeTransitions::RISING:
    case machinekit_interfaces::ProbeTransitions::FALLING:
        // Do special handling below for probe moves
        checkReachedTrajectoryGoalProbe();
    default:
        // Normal moves get forwarded to the stock goal check
        JointTrajectoryControllerType::checkReachedTrajectoryGoal();
    }
}

template <class SegmentImpl, class HardwareInterface>
void InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
checkReachedTrajectoryGoalProbe()
{
    //If there is an active goal and all segments finished successfully then set goal as succeeded
    RealtimeGoalHandlePtr current_active_goal(this->rt_active_goal_);
    if (current_active_goal && this->successful_joint_traj_.count() == this->getNumberOfJoints())
    {
        // FIXME do we need a dedicated error code for this in the message?
        current_active_goal->preallocated_result_->error_code = -7;
        current_active_goal->setAborted(current_active_goal->preallocated_result_);
        current_active_goal.reset(); // do not publish feedback
        this->rt_active_goal_.reset();
        this->successful_joint_traj_.reset();
    }
}

template <class SegmentImpl, class HardwareInterface>
bool InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::handleProbeRequest(stop_event_msgs::SetNextProbeMoveRequest &request, stop_event_msgs::SetNextProbeMoveResponse &response)
{
    probe_handle.setProbeCapture(request.mode);
    // TODO more informative message
    response.message="Probing mode set";
    response.success=true;
    return true;
}

template <class SegmentImpl, class HardwareInterface>
bool InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::handleProbeResultRequest(stop_event_msgs::GetProbeResultRequest &request, stop_event_msgs::GetProbeResultResponse &response)
{
    auto &probe_state = response.result.event_state;
    for (unsigned int joint_index = 0; joint_index < this->getNumberOfJoints(); ++joint_index)
    {
        // Hope that the lock thrashing here doesn't affect RT...
        probe_state.positions[joint_index] = probe_joint_results_[joint_index].getPosition();
        probe_state.velocities[joint_index] = probe_joint_results_[joint_index].getVelocity();
    }
    // TODO fill in the rest of this stuff
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
          const std::string err_msg("Can't start a motion or probe move with probe active");
          if (error_string) {
            *error_string = err_msg;
          }
          return false;
        }
        }
    }
    return JointTrajectoryControllerType::updateTrajectoryCommand(msg, gh, error_string);
}


} // namespace
