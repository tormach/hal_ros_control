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
/**
 * \brief A `ros_control_boilerplate::GenericHWControlLoop`-like class for
 * Machinekit HAL
 *
 * Implements the ROS node and ros_control `read()`/`update()`/`write()` loop
 * running in a Machinekit HAL component
 *
 * This class does the messy work of managing a C++ ROS node linked into a C HAL
 * component.  It sets up the control loop object in the component's
 * `rtapi_app_main()` init function, runs the ros_control `read(); update();
 * write()` in the update function, and finally shuts down the node and
 * component.
 *
 * Most of the real time work is done in the `hal_hw_interface::HalHWInterface`
 * class.
 */

class HalRosControlLoop
{
public:
  /**
   * \brief Constructor
   *
   */
  HalRosControlLoop();

  /**
   * \brief Destructor
   *
   * Calls `shutdown()`
   */
  ~HalRosControlLoop();

  /**
   * \brief Initialize control loop object
   *
   * * Set up the ROS node
   * * Run the ROS spinner thread
   * * Initialize the `hal_hw_interface::HalHWInterface` object
   * * Initialize the `controller_manager::ControllerManager` object
   */
  int init();

  /**
   * \brief Run one ros_control `read()/update()/write()` cycle
   *
   * This implements HAL component function, periodically run in a HAL real time
   * thread.  This is wrapped in `funct()` for registering the C-linkable HAL
   * component function callback at HAL component creation.
   */
  void update(long period);

  /**
   * \brief Shut down the robot hardware interface & controller
   *
   * Shuts down the ROS node and HAL component
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

}  // namespace hal_hw_interface

#endif  // HAL_HW_INTERFACE_HAL_CONTROL_LOOP_H
