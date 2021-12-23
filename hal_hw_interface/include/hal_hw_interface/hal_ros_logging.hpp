// Copyright (c) 2018, John Morris
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//     * Redistributions of source code must retain the above
//       copyright notice, this list of conditions and the following
//       disclaimer.
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials
//       provided with the distribution.
//     * Neither the name of the <organization> nor the names of its
//       contributors may be used to endorse or promote products
//       derived from this software without specific prior written
//       permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
// <COPYRIGHT HOLDER> BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
// USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
// SUCH DAMAGE.

#ifndef HAL_HW_INTERFACE__HAL_ROS_LOGGING_HPP_
#define HAL_HW_INTERFACE__HAL_ROS_LOGGING_HPP_

// HAL
#define RTAPI 1
#include <hal.h>

// ROS
#include <rclcpp/logging.hpp>

#define HAL_ROS_LOG(hal_lev, ros_lev, logger, ...)                             \
  do                                                                           \
  {                                                                            \
    RCLCPP_##ros_lev(logger, __VA_ARGS__);                                     \
    hal_print_msg(RTAPI_MSG_##hal_lev, __VA_ARGS__);                           \
  } while (0)

#define HAL_ROS_DBG(logger, ...) HAL_ROS_LOG(DBG, DEBUG, logger, __VA_ARGS__)
#define HAL_ROS_INFO(logger, ...) HAL_ROS_LOG(INFO, INFO, logger, __VA_ARGS__)
#define HAL_ROS_ERR(logger, ...) HAL_ROS_LOG(ERR, ERROR, logger, __VA_ARGS__)

#define HAL_ROS_LOG_NAMED(hal_lev, ros_lev, name, ...)                         \
  do                                                                           \
  {                                                                            \
    RCUTILS_LOG_##ros_lev##_NAMED(name, __VA_ARGS__);                          \
    hal_print_msg(RTAPI_MSG_##hal_lev, __VA_ARGS__);                           \
  } while (0)

#define HAL_ROS_DBG_NAMED(name, ...)                                           \
  HAL_ROS_LOG_NAMED(DBG, DEBUG, name, __VA_ARGS__)
#define HAL_ROS_INFO_NAMED(name, ...)                                          \
  HAL_ROS_LOG_NAMED(INFO, INFO, name, __VA_ARGS__)
#define HAL_ROS_ERR_NAMED(name, ...)                                           \
  HAL_ROS_LOG_NAMED(ERR, ERROR, name, __VA_ARGS__)

#endif  // HAL_HW_INTERFACE__HAL_ROS_LOGGING_HPP_
