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

#ifndef HAL_HW_INTERFACE_HAL_CONTROL_LOOP_H
#define HAL_HW_INTERFACE_HAL_CONTROL_LOOP_H

#include <hal_hw_interface/hal_hw_interface.h>
#include <controller_manager/controller_manager.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>

namespace hal_hw_interface
{
class HalRosControlLoop
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   * \param hardware_interface - Hardware interface object
   */
  HalRosControlLoop();

  /**
   * \brief Destructor
   */
  ~HalRosControlLoop();

  /**
   * \brief HAL RT component function that runs one ros_control
   * read()/update()/write() cycle; this is wrapped in funct() for
   * registering the C-linkable HAL component function callback at
   * HAL component creation
   */
  void update(long period);

  /**
   * \brief Shut down the robot hardware interface & controller
   */
  void shutdown();

protected:
  /**
   * \brief The ROS node handle
   */
  boost::shared_ptr<ros::NodeHandle> nh_;

  /////// Non-RT CB thread pieces ///////
  /**
   * \brief The non-RT ROS thread callback function
   */
  void serviceNonRtRosQueue();

  // - Non-RT thread and callback queue
  boost::thread non_rt_ros_queue_thread_;
  ros::CallbackQueue non_rt_ros_queue_;

  /////// RT pieces ///////

  /**
   * \brief The HAL hardware interface
   */
  boost::shared_ptr<hal_hw_interface::HalHWInterface> hardware_interface_;

  /**
   * \brief The ros_control controller_manager
   */
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  /**
   * \brief Flag indicating node is shut down
   */
  bool node_is_shutdown;

};  // class

}  // namespace

#endif  // HAL_HW_INTERFACE_HAL_CONTROL_LOOP_H
