#include "interruptible_joint_trajectory_controller/feedhold_handler.h"

FeedholdHandler::FeedholdHandler(
    boost::shared_ptr<ros::NodeHandle> controller_nh)
  : controller_nh_(controller_nh)
  , feedhold_(false)  // Initialize to false by default
{
  const std::string service_name =
      velocity_override_msgs::ServiceNames::FEEDHOLD_SERVICE_NAME;
  service_server_ = controller_nh_->advertiseService(
      service_name, &FeedholdHandler::feedholdServiceCallback, this);
}

FeedholdHandler::~FeedholdHandler()
{
  // Destructor logic here, if needed
}

bool FeedholdHandler::feedholdServiceCallback(
    velocity_override_msgs::FeedholdStatusService::Request& req,
    velocity_override_msgs::FeedholdStatusService::Response& res)
{
  feedhold_ = req.feedhold_setting;
  if (feedhold_)
  {
    updateTargetScalingFactor(0.0);
    ROS_INFO("Feedhold activated. Scaling factor set to 0.0");
  }
  else
  {
    updateTargetScalingFactor(1.0);
    ROS_INFO("Feedhold deactivated. Scaling factor set to 1.0");
  }

  double remaining_transition_time =
      rt_transition_time_.load() * TRANSITION_TIME_SAFETY_FACTOR;

  while (feedhold_)
  {
    // motion execution has stopped
    if (getCurrentScalingFactor() < 0.0001)
    {
      break;
    }
    double time_increment = 0.01;
    ros::Duration(time_increment).sleep();
    remaining_transition_time -= time_increment;

    if (remaining_transition_time < 0.0)
    {
      ROS_WARN("Feedhold transition time exceeded.");
      res.success = false;
      return true;
    }
  }

  res.success = true;
  return true;
}

bool FeedholdHandler::getFeedholdStatus() const
{
  return feedhold_;
}
