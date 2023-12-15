#ifndef ACTIVE_MOVE_HANDLER_H
#define ACTIVE_MOVE_HANDLER_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <velocity_override_msgs/MoveTypeService.h>
#include <velocity_override_msgs/MoveTypes.h>
#include <velocity_override_msgs/ServiceNames.h>

class ActiveMoveHandler
{
public:
  explicit ActiveMoveHandler(boost::shared_ptr<ros::NodeHandle> controller_nh);
  ~ActiveMoveHandler();

  uint8_t getMoveType() const;
  double getVelocityScale() const;

private:
  bool moveTypeServiceCallback(
      velocity_override_msgs::MoveTypeService::Request& req,
      velocity_override_msgs::MoveTypeService::Response& res);

  boost::shared_ptr<ros::NodeHandle> controller_nh_;
  ros::ServiceServer service_server_;

  uint8_t moveType_;
  double velocityScale_;
};

#endif  // ACTIVE_MOVE_HANDLER_H
