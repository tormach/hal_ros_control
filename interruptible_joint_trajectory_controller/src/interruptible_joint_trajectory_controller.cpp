///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
// Copyright (c) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of PAL Robotics S.L. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
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

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

// Project
#include <trajectory_interface/quintic_spline_segment.h>
#include <interruptible_joint_trajectory_controller/interruptible_joint_trajectory_controller.h>

namespace position_controllers
{
  /**
   * \brief Joint trajectory controller that represents trajectory segments as <b>quintic splines</b> and sends
   * commands to a \b position interface.
   */
  typedef interruptible_joint_trajectory_controller::InterruptibleJointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                                 hardware_interface::PositionJointInterface>
          InterruptibleJointTrajectoryController;
}

namespace velocity_controllers
{
  /**
   * \brief Joint trajectory controller that represents trajectory segments as <b>quintic splines</b> and sends
   * commands to a \b velocity interface.
   */
  typedef interruptible_joint_trajectory_controller::InterruptibleJointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                                 hardware_interface::VelocityJointInterface>
          InterruptibleJointTrajectoryController;
}

namespace effort_controllers
{
  /**
   * \brief Joint trajectory controller that represents trajectory segments as <b>quintic splines</b> and sends
   * commands to an \b effort interface.
   */
  typedef interruptible_joint_trajectory_controller::InterruptibleJointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                                 hardware_interface::EffortJointInterface>
          InterruptibleJointTrajectoryController;
}

namespace pos_vel_controllers
{
  /**
   * \brief Joint trajectory controller that represents trajectory segments as <b>quintic splines</b> and sends
   * commands to an \b pos_vel interface.
   */
  typedef interruptible_joint_trajectory_controller::InterruptibleJointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                                 hardware_interface::PosVelJointInterface>
          InterruptibleJointTrajectoryController;
}

namespace pos_vel_acc_controllers
{
  /**
   * \brief Joint trajectory controller that represents trajectory segments as <b>quintic splines</b> and sends
   * commands to a \b pos_vel_acc interface.
   */
  typedef interruptible_joint_trajectory_controller::InterruptibleJointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                                 hardware_interface::PosVelAccJointInterface>
          InterruptibleJointTrajectoryController;
}

PLUGINLIB_EXPORT_CLASS(position_controllers::InterruptibleJointTrajectoryController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(velocity_controllers::InterruptibleJointTrajectoryController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(effort_controllers::InterruptibleJointTrajectoryController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(pos_vel_controllers::InterruptibleJointTrajectoryController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(pos_vel_acc_controllers::InterruptibleJointTrajectoryController, controller_interface::ControllerBase)
