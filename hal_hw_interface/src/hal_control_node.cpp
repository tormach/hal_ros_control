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

#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>
#include <hal_hw_interface/hal_ros_logging.hpp>
#include <hal_hw_interface/hal_def.hpp>
#include <memory>
#include <string>

// Controller manager pointer
std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

// ROS asynch executor thread pointer
std::shared_ptr<std::thread> executor_thread;

extern "C" {
// Pre-declare the HAL function
void funct(void* arg, int64_t period);

int rtapi_app_main(void)
{
  // Init ROS node, delegating signal handling to rtapi_app
  auto init_opts = rclcpp::InitOptions();
  init_opts.shutdown_on_sigint = false;
  rclcpp::init(0, NULL, init_opts);

  // Init HAL component
  int comp_id_ = hal_init(CNAME);
  if (comp_id_ < 0)
  {
    HAL_ROS_ERR_NAMED(CNAME, "%s:  ERROR: Component creation ABORTED", CNAME);
    // return false; // FIXME
    return -1;
  }

  HAL_ROS_INFO_NAMED(CNAME, "%s: Initialized HAL component", CNAME);

  // Init controller manager and asynch executor
  // - Resource manager & hardware interface loaded here; HAL pins initialized
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  std::string manager_node_name = "controller_manager";
  controller_manager_.reset(
      new controller_manager::ControllerManager(executor, manager_node_name));

  // Launch executor in asynch thread
  executor_thread.reset(new std::thread([&executor]() { executor->spin(); }));

  // Export the function & mark component ready
  if (hal_export_functf(funct, NULL, 1, 0, comp_id_, "%s.funct", CNAME) < 0)
  {
    HAL_ROS_INFO_NAMED(CNAME, "ERROR: hal_export_functf failed");
    hal_exit(comp_id_);
    return -1;
  }
  hal_ready(comp_id_);

  HAL_ROS_INFO_NAMED(CNAME, "%s:  HAL component ready!", CNAME);

  rclcpp::Logger l = controller_manager_->get_logger();
  HAL_ROS_INFO(l, "HAL controller manager initializing");

  return 0;  // Success
}

void funct([[maybe_unused]] void* arg, [[maybe_unused]] int64_t period)
{
  controller_manager_->read();
  controller_manager_->update();
  controller_manager_->write();
}

void rtapi_app_exit(void)
{
  rclcpp::Logger l = controller_manager_->get_logger();
  HAL_ROS_INFO(l, "HAL controller manager shutting down");
  executor_thread->join();
}

}  // extern "C"
