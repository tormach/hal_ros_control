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

#include <hal_hw_interface/hal_control_loop.h>
#include <hal_hw_interface/hal_ros_logging.h>

#include <cstdlib>  // getenv()

// Pre-declare the HAL function
extern "C" void funct(void* arg, long period);

namespace hal_hw_interface
{
HalRosControlLoop::HalRosControlLoop() : node_is_shutdown(0)
{
  // ROS node handle
  nh_.reset(new ros::NodeHandle(""));

  // Run ROS loop in a separate thread
  ros::AsyncSpinner spinner(2);
  spinner.start();

  std::string ros_master_uri = getenv("ROS_MASTER_URI");

  HAL_ROS_LOG_INFO(CNAME,
                   "%s: Initialized ROS node and spinner; ROS_MASTER_URI = %s",
                   ros_master_uri.c_str(), CNAME);

  // HW interface
  hardware_interface_.reset(new hal_hw_interface::HalHWInterface(*nh_));

  HAL_ROS_LOG_INFO(CNAME, "%s: Initialized HAL hardware interface", CNAME);

  // ROS callback thread
  nh_->setCallbackQueue(&non_rt_ros_queue_);
  non_rt_ros_queue_thread_ = boost::thread(
      boost::bind(&HalRosControlLoop::serviceNonRtRosQueue, this));

  HAL_ROS_LOG_INFO(CNAME, "%s: Done initializing ROS callback thread", CNAME);

  // Controller
  controller_manager_.reset(new controller_manager::ControllerManager(
      hardware_interface_.get(), *nh_));

  HAL_ROS_LOG_INFO(CNAME, "%s: Done initializing ROS controller manager",
                   CNAME);

  // Init HAL hardware interface
  hardware_interface_->init_hal(&funct);

  HAL_ROS_LOG_INFO(CNAME, "%s: Done initializing HAL hardware interface",
                   CNAME);

  HAL_ROS_LOG_INFO(CNAME, "HAL control loop ready.");
}  // constructor

// Non-RT thread CB function
void HalRosControlLoop::serviceNonRtRosQueue()
{
  static const double timeout = 0.001;

  while (this->nh_->ok())
    this->non_rt_ros_queue_.callAvailable(ros::WallDuration(timeout));
}

void HalRosControlLoop::shutdown()
{
  // Only run once
  if (node_is_shutdown)
  {
    HAL_ROS_LOG_INFO(CNAME, "%s: shutdown() called again", CNAME);
    return;
  }
  node_is_shutdown = 1;

  // Shut down ROS node
  HAL_ROS_LOG_INFO(CNAME, "%s:  Shutting down ROS node", CNAME);
  nh_->shutdown();
  HAL_ROS_LOG_INFO(CNAME, "%s:  Shutting down ROS cb thread", CNAME);
  non_rt_ros_queue_thread_.join();

  // Shut down HAL
  HAL_ROS_LOG_INFO(CNAME, "%s:  Shutting down HAL", CNAME);
  hardware_interface_->shutdown();

  // Reset shared pointers
  controller_manager_.reset();
  hardware_interface_.reset();
  nh_.reset();

  HAL_ROS_LOG_INFO(CNAME, "%s:  Shut down complete", CNAME);
}

HalRosControlLoop::~HalRosControlLoop()
{
  shutdown();
}

// This and HAL basically replicate generic_hw_control_loop.cpp
void HalRosControlLoop::update(long period)
{
  ros::Duration ros_period(period / 1000000000ull, period % 1000000000ull);

  hardware_interface_->read(ros_period);
  // For debugging; spews at startup
  // if (hardware_interface_->reset_controllers)
  //   // don't use ROS logging in RT context
  //   rtapi_print_msg(RTAPI_MSG_INFO, "%s:  Resetting controllers", CNAME);
  controller_manager_->update(ros::Time::now(), ros_period,
                              hardware_interface_->reset_controllers);
  hardware_interface_->write(ros_period);
}

}  // namespace

// The HAL hardware interface control loop object
boost::shared_ptr<hal_hw_interface::HalRosControlLoop> control_loop_;

extern "C" {
int rtapi_app_main(void)
{
  // Init ROS node
  // - Pass ROS_MASTER_URI (none causes a sigsegv on ros::init())
  // - Delegate signal handling to rtapi_app
  const ros::M_string remappings = {
    { "__master", getenv("ROS_MASTER_URI") },
  };
  ros::init(remappings, CNAME, ros::init_options::NoSigintHandler);

  // Create HAL controller and hardware interface
  control_loop_.reset(new hal_hw_interface::HalRosControlLoop());

  return 0;
}

void funct(void* arg, long period)
{
  control_loop_->update(period);
}

void rtapi_app_exit(void)
{
  control_loop_->shutdown();
  // hw_interface_ smart pointer doesn't need to be deleted
}

}  // extern "C"
