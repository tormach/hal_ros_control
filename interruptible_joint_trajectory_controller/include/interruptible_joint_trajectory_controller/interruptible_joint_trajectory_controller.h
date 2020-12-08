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

// ros_controls
#include <realtime_tools/realtime_server_goal_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/internal/demangle_symbol.h>

#include <machinekit_interfaces/realtime_event_interface.h>

// Project
#include <joint_trajectory_controller/joint_trajectory_controller.h>

namespace interruptible_joint_trajectory_controller
{

/**
 * Controller for executing joint-space trajectories on a group of joints, that can respond to stop events triggered from hardware in the realtime loop.
 *
 * See JointTrajectoryController documentation for details on how the trajectory execution works.
 *
 */
template <class SegmentImpl, class HardwareInterface>
class InterruptibleJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>
{
public:

  InterruptibleJointTrajectoryController();

  /** \name Non Real-Time Safe Functions
   *\{*/
  bool init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  /*\}*/

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Holds the current position. */
  void starting(const ros::Time& time);

  /** \brief Cancels the active action goal, if any. */
  void stopping(const ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);
  /*\}*/

protected:
  using JointTrajectoryControllerType = typename joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>;

  // TODO

  // For communication of stop events and controller configuration
  ros::Subscriber    probe_trajectory_command_sub_;
  // TODO ROS service interface here
  ros::Publisher     stop_event_notification_;

private:
  /**
   * @brief Updates the pre-allocated feedback of the current active goal (if any)
   * based on the current state values.
   *
   * @note This function is NOT thread safe but intended to be used in the
   * update-function.
   */
  void setActionFeedback();

};

} // namespace

#pragma once

namespace interruptible_joint_trajectory_controller
{

template <class SegmentImpl, class HardwareInterface>
inline void InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
starting(const ros::Time& time)
{
    static_cast<JointTrajectoryControllerType*>(this)->starting(time);
    // TODO Custom probe behavior here
}

template <class SegmentImpl, class HardwareInterface>
inline void InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
stopping(const ros::Time& time)
{
    static_cast<JointTrajectoryControllerType*>(this)->stopping(time);
    // TODO custom probe behavior here
}

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
    // TODO custom behavior
    return static_cast<JointTrajectoryControllerType*>(this)->init(hw, root_nh, controller_nh);
}

template <class SegmentImpl, class HardwareInterface>
void InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
update(const ros::Time& time, const ros::Duration& period)
{
    // TODO check probe state and trigger a stop if it's hit
    return static_cast<JointTrajectoryControllerType*>(this)->update(time, period);
}

} // namespace
