#include "interruptible_joint_trajectory_controller/active_move_handler.h"
#include <set>

ActiveMoveHandler::ActiveMoveHandler(
    boost::shared_ptr<ros::NodeHandle> controller_nh)
  : controller_nh_(controller_nh)
  , moveType_(velocity_override_msgs::MoveTypes::JOG)
  , velocityScale_(double(1.0))
  , move_id_(-1)
{
  const std::string service_name =
      velocity_override_msgs::ServiceNames::NEXT_MOVE_SERVICE_NAME;
  service_server_ = controller_nh_->advertiseService(
      service_name, &ActiveMoveHandler::moveTypeServiceCallback, this);
}

ActiveMoveHandler::~ActiveMoveHandler()
{
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
    ROS_INFO("Received move type: PROGRAM_MOVE PROCESSING");
    move_id_++;
    if(move_id_handle_){
      move_id_handle_->set(move_id_);
    }
    ROS_INFO("Received move type: PROGRAM_MOVE move_id: %d", move_id_);
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

void ActiveMoveHandler::setMoveIdHandle(machinekit_interfaces::HALS32PinHandle& handle) {
  move_id_handle_ = &handle;
}

int ActiveMoveHandler::getMoveId() const {
  return move_id_;
}