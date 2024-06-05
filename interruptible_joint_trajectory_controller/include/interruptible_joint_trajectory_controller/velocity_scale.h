#ifndef VELOCITY_SCALE_H
#define VELOCITY_SCALE_H

#include <atomic>
#include <cmath>  // for std::abs
#include <string>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <redis_store_msgs/ParamUpdate.h>
#include "interruptible_joint_trajectory_controller/velocity_scale_base.h"

class VelocityScale : public VelocityScaleBase
{
public:
  static constexpr double TRANSITION_TIME_INTERPOLATION_TRESH = 0.010;
  // TODO: make this const
  const std::string CONFIG_MANAGER_TOPIC_NAME{ "/config_manager/update" };

  explicit VelocityScale(
      boost::shared_ptr<ros::NodeHandle> controller_nh,
      const std::string& scale_factor_param_name);  // constructor

  ~VelocityScale()
  {
    // ROS_WARN("VelocityScale DESTRUCTOR");
  }

  void paramUpdateCallback(const redis_store_msgs::ParamUpdate::ConstPtr& msg);

  const std::string SCALE_FACTOR_PARAM_NAME;

private:
  boost::shared_ptr<ros::NodeHandle> controller_nh_;
  ros::Subscriber slider_subscriber_;
};

#endif  // VELOCITY_SCALE_H
