#include "interruptible_joint_trajectory_controller/active_move_handler.h"

ActiveMoveHandler::ActiveMoveHandler(
    boost::shared_ptr<ros::NodeHandle> controller_nh)
  : controller_nh_(controller_nh)
  , moveType_(velocity_override_msgs::MoveTypes::JOG)
  , velocityScale_(double(1.0))
{
  const std::string service_name =
      velocity_override_msgs::ServiceNames::NEXT_MOVE_SERVICE_NAME;
  service_server_ = controller_nh_->advertiseService(
      service_name, &ActiveMoveHandler::moveTypeServiceCallback, this);
}

ActiveMoveHandler::~ActiveMoveHandler()
{
  // Destructor logic here, if needed
}

bool ActiveMoveHandler::moveTypeServiceCallback(
    velocity_override_msgs::MoveTypeService::Request& req,
    velocity_override_msgs::MoveTypeService::Response& res)
{
  if (req.move_type != velocity_override_msgs::MoveTypes::PROGRAM_MOVE &&
      req.move_type != velocity_override_msgs::MoveTypes::JOG)
  {
    ROS_WARN("Received an invalid move_type: %u", req.move_type);
    res.success = false;
    return true;
  }

  if (req.move_type == velocity_override_msgs::MoveTypes::PROGRAM_MOVE)
  {
    ROS_INFO("Received move type: PROGRAM_MOVE");
  }
  else if (req.move_type == velocity_override_msgs::MoveTypes::JOG)
  {
    ROS_INFO("Received move type: JOG");
  }

  moveType_ = req.move_type;
  velocityScale_ = req.velocity_scale;

  res.success = true;
  return true;
}

uint8_t ActiveMoveHandler::getMoveType() const
{
  return moveType_;
}

double ActiveMoveHandler::getVelocityScale() const
{
  return velocityScale_;
}
