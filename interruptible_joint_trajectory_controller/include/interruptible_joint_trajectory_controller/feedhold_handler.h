#ifndef FEEDHOLD_HANDLER_H
#define FEEDHOLD_HANDLER_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <velocity_override_msgs/FeedholdStatusService.h>
#include <velocity_override_msgs/ServiceNames.h>
#include "interruptible_joint_trajectory_controller/velocity_scale_base.h"

class FeedholdHandler : public VelocityScaleBase
{
public:
  explicit FeedholdHandler(boost::shared_ptr<ros::NodeHandle> controller_nh);
  ~FeedholdHandler();

  bool getFeedholdStatus() const;

private:
  static constexpr double TRANSITION_TIME_SAFETY_FACTOR = 1.200;

  bool feedholdServiceCallback(
      velocity_override_msgs::FeedholdStatusService::Request& req,
      velocity_override_msgs::FeedholdStatusService::Response& res);

  boost::shared_ptr<ros::NodeHandle> controller_nh_;
  ros::ServiceServer service_server_;

  bool feedhold_;
};

#endif  // FEEDHOLD_HANDLER_H
