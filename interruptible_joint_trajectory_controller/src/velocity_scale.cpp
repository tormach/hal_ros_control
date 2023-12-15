#include "interruptible_joint_trajectory_controller/velocity_scale.h"

// const char* VelocityScale::CONFIG_MANAGER_TOPIC_NAME =
// "/config_manager/update";

// Constructor
// VelocityScale::VelocityScale(ros::NodeHandle& controller_nh, const
// std::string& scale_factor_param_name)
//     : SCALE_FACTOR_PARAM_NAME(scale_factor_param_name),
//     controller_nh_(controller_nh) {
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

  // std::string nodeName = ros::this_node::getName();
  // ROS_WARN("Node name is: %s", nodeName.c_str());
  // ROS_WARN("IN CONTROLLER STORE: %s received %f",
  // SCALE_FACTOR_PARAM_NAME.c_str(), initial_slider_value);
  // ROS_WARN("CONFIG_MANAGER_TOPIC_NAME IS: %s",
  // CONFIG_MANAGER_TOPIC_NAME.c_str());
}

void VelocityScale::paramUpdateCallback(
    const redis_store_msgs::ParamUpdate::ConstPtr& msg)
{
  // ROS_WARN("IN CONTROLLER OUTSIDE CONDITION %s received %s",
  // SCALE_FACTOR_PARAM_NAME.c_str(), msg->param_name.c_str());
  // ROS_WARN("Callback entered, param_name: %s", msg->param_name.c_str());

  // ROS_WARN("IN CONTROLLER OUTSIDE CONDITION %s received %s",
  // SCALE_FACTOR_PARAM_NAME, msg->param_name);
  if (msg->param_name == SCALE_FACTOR_PARAM_NAME)
  {
    // Convert the string param_value to double
    double target_factor = std::atof(msg->param_value.c_str());
    updateTargetScalingFactor(target_factor);
    // ROS_INFO_STREAM("IN CONTROLLER %s updated: %f",
    // SCALE_FACTOR_PARAM_NAME.c_str(), scaling_factor); ROS_WARN("IN CONTROLLER
    // " << SCALE_FACTOR_PARAM_NAME << " updated: " << scaling_factor);
    // ROS_WARN("IN CONTROLLER %s updated: %f", SCALE_FACTOR_PARAM_NAME.c_str(),
    // target_factor);
  }
}
