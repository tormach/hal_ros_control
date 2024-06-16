#ifndef VELOCITY_SCALE_MANAGER_H
#define VELOCITY_SCALE_MANAGER_H

#include <vector>
#include <memory>
#include <atomic>

#include <redis_store_msgs/ParamUpdate.h>
#include <std_msgs/Float64.h>
#include <velocity_override_msgs/MoveTypes.h>
#include <boost/shared_ptr.hpp>

#include "interruptible_joint_trajectory_controller/velocity_scale.h"
#include "interruptible_joint_trajectory_controller/active_move_handler.h"
#include "interruptible_joint_trajectory_controller/feedhold_handler.h"

class VelocityScaleManager
{
public:
  // Constructor
  VelocityScaleManager() = default;
  explicit VelocityScaleManager(
      boost::shared_ptr<ros::NodeHandle> controller_nh);
  VelocityScaleManager& operator=(VelocityScaleManager&& other) noexcept
  {
    return *this;
  }

  void updateVelocityScales(double period) const;
  void updateTransitionTimes(double transition_time_s) const;
  double getCurrentScalingFactor() const;

  std::shared_ptr<VelocityScale> uniform_velocity_scale_;
  std::shared_ptr<VelocityScale> maxvel_scale_;

private:
  const std::string MAXVEL_SCALE_PARAMETER = "user_config/maximum_velocity_scale";
  const std::string UNIFORM_VEL_SCALE_PARAMETER = "user_config/"
                                                  "uniform_velocity_scale";
  const std::string VEL_TRANSITION_TIME_TOPIC_NAME = "/velocity_"
                                                     "transition_time";

  ros::Subscriber velocity_transition_time_subscriber_;
  void transitionTimeUpdateCallback(const std_msgs::Float64::ConstPtr& msg);

  boost::shared_ptr<ros::NodeHandle> nh_ptr_;

  std::shared_ptr<FeedholdHandler> feedhold_handler_;
  std::shared_ptr<ActiveMoveHandler> active_move_handler_;
};

#endif  // VELOCITY_SCALE_MANAGER_H
