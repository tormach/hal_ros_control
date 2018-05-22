#ifndef HAL_HW_INTERFACE_HAL_ROS_LOGGING_H
#define HAL_HW_INTERFACE_HAL_ROS_LOGGING_H

#include <hal.h>
#include <ros/console.h>

#define HAL_ROS_LOG(hal_lev, ros_lev, name, ...)        \
  do {                                                  \
    ROS_ ## ros_lev ## _NAMED(name, __VA_ARGS__);       \
    hal_print_msg(RTAPI_MSG_ ## hal_lev, __VA_ARGS__);  \
  } while (0)

#define HAL_ROS_LOG_DBG(name, ...)              \
  HAL_ROS_LOG(DBG, DBG, name, __VA_ARGS__)

#define HAL_ROS_LOG_INFO(name, ...)             \
  HAL_ROS_LOG(INFO, INFO, name, __VA_ARGS__)

#define HAL_ROS_LOG_ERR(name, ...)              \
  HAL_ROS_LOG(ERR, ERROR, name, __VA_ARGS__)

#endif // HAL_HW_INTERFACE_HAL_ROS_LOGGING_H
