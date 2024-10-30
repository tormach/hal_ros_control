#include "interruptible_joint_trajectory_controller/velocity_scale_manager.h"

// Constructor
// VelocityScaleManager::VelocityScaleManager(ros::NodeHandle& controller_nh) {
VelocityScaleManager::VelocityScaleManager(
    boost::shared_ptr<ros::NodeHandle> controller_nh)
  : nh_ptr_(controller_nh)  // Store the shared_ptr for later use
{
  maxvel_scale_ =
      std::make_shared<VelocityScale>(controller_nh, MAXVEL_SCALE_PARAMETER);
  uniform_velocity_scale_ = std::make_shared<VelocityScale>(
      controller_nh, UNIFORM_VEL_SCALE_PARAMETER);
  feedhold_handler_ = std::make_shared<FeedholdHandler>(controller_nh);
  active_move_handler_ = std::make_shared<ActiveMoveHandler>(controller_nh);
  // safety_pin_interface_ = machinekit_interfaces::HALBitPinInterface();

  // safety_pin_interface_.registerHandle();

  // Subscribe to ROS topics
  velocity_transition_time_subscriber_ = controller_nh->subscribe(
      VEL_TRANSITION_TIME_TOPIC_NAME, 10,
      &VelocityScaleManager::transitionTimeUpdateCallback, this);
}

void VelocityScaleManager::updateVelocityScales(double period) const
{
  // speed-up velocity scale transition for slow moves
  // expected value from range [1.0, 10.0]
  double velocity_scale_compensation_factor =
      1.0 / active_move_handler_->getVelocityScale();
  if (velocity_scale_compensation_factor > 10.0)
  {
    velocity_scale_compensation_factor = 10.0;
  }
  maxvel_scale_->update(period * velocity_scale_compensation_factor);
  uniform_velocity_scale_->update(period * velocity_scale_compensation_factor);
  feedhold_handler_->update(period * velocity_scale_compensation_factor);
}

void VelocityScaleManager::updateTransitionTimes(double transition_time_s) const
{
  maxvel_scale_->updateTransitionTime(transition_time_s);
  uniform_velocity_scale_->updateTransitionTime(transition_time_s);
  feedhold_handler_->updateTransitionTime(transition_time_s);
}

void VelocityScaleManager::transitionTimeUpdateCallback(
    const std_msgs::Float64::ConstPtr& msg)
{
  if (msg->data < 0.0 || msg->data > 1.0)
  {
     ROS_ERROR("IN CONTROLLER Scaling factor is not valid: %f", msg->data);
    return;
  }
  updateTransitionTimes(msg->data);
}

double VelocityScaleManager::getCurrentScalingFactor() const
{
  int current_move_type = active_move_handler_->getMoveType();

  // if(current_move_type == velocity_override_msgs::MoveTypes::PROGRAM_MOVE ||
  // true)
  if (current_move_type == velocity_override_msgs::MoveTypes::PROGRAM_MOVE)
  {
    double move_velocity_scale = active_move_handler_->getVelocityScale();
    double current_maxvel_limit = maxvel_scale_->getCurrentScalingFactor();

    double scale_factor_candiate =
        uniform_velocity_scale_->getCurrentScalingFactor() *
        feedhold_handler_->getCurrentScalingFactor();

    // flatten the scaling factor if it is above the maxvel limit
    if (move_velocity_scale > current_maxvel_limit)
    {
      scale_factor_candiate *= current_maxvel_limit / move_velocity_scale;
    }
    return scale_factor_candiate;
  }
  else if (current_move_type == velocity_override_msgs::MoveTypes::JOG)
  {
    // technically cant be jogging when in feedhold,
    // but applying multipler just in case
    return 1.0 * feedhold_handler_->getCurrentScalingFactor();
  }
  else
  {
    ROS_ERROR_STREAM("Unknown move type: " << current_move_type);
    return 0.0;
  }
}
