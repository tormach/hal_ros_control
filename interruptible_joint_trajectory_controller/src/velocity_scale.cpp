#include "interruptible_joint_trajectory_controller/velocity_scale.h"

VelocityScale::VelocityScale(boost::shared_ptr<ros::NodeHandle> controller_nh,
                             const std::string& scale_factor_param_name)
  : SCALE_FACTOR_PARAM_NAME(scale_factor_param_name)
  , controller_nh_(controller_nh)
{
  slider_subscriber_ = controller_nh_->subscribe(
      CONFIG_MANAGER_TOPIC_NAME, 10, &VelocityScale::paramUpdateCallback, this);

  double initial_slider_value;
  std::string prefixed_param_name = "/" + SCALE_FACTOR_PARAM_NAME;
  if (!controller_nh_->getParam(prefixed_param_name, initial_slider_value))
  {
    ROS_WARN("Failed to get parameter %s. Using default value.",
             SCALE_FACTOR_PARAM_NAME.c_str());
    initial_slider_value = 1.0;  // Default value
  }
  rt_target_scaling_factor_.store(initial_slider_value);
}

void VelocityScale::paramUpdateCallback(
    const redis_store_msgs::ParamUpdate::ConstPtr& msg)
{
  if (msg->param_name == SCALE_FACTOR_PARAM_NAME)
  {
    // Convert the string param_value to double
    double target_factor = std::atof(msg->param_value.c_str());
    updateTargetScalingFactor(target_factor);
  }
}
