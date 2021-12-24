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

#include <stdlib.h>
#include <pthread.h>  // pthread_setname_np()
#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>
#include <hal_hw_interface/hal_ros_logging.hpp>
#include <hal_hw_interface/hal_def.hpp>
#include <memory>
#include <string>

// Controller manager node and executor pointers
std::shared_ptr<controller_manager::ControllerManager> CONTROLLER_MANAGER;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> EXECUTOR;

extern "C" {
// Pre-declare the HAL function
void funct(void* arg, int64_t period);

#define MAX_ARGS 255
static char* ARGV[MAX_ARGS] = {
  nullptr,
};
RTAPI_MP_ARRAY_STRING(ARGV, MAX_ARGS, "ROS command-line argv");

int rtapi_app_main(void)
{
  // Init HAL component
  int comp_id = hal_init(CNAME);
  if (comp_id < 0)
  {
    HAL_ROS_ERR_NAMED(CNAME, "%s:  ERROR: Component creation ABORTED", CNAME);
    // return false; // FIXME
    return -1;
  }

  HAL_ROS_INFO_NAMED(CNAME, "%s: Initialized HAL component %d", CNAME, comp_id);

  // Apparently rcl searches through $LD_LIBRARY_PATH to find
  // librmw_fastrtps_cpp.so and related libs.  :P Since HAL's rtapi_app runs
  // setuid, $LD_LIBRARY_PATH is ignored, so we have to artificially reconstruct
  // it.  Can't go through the ROS params, since no node yet, so fake it with
  // the single /opt/ros/$ROS_DISTRO/lib entry.
  char ld_library_path_buf[256];
  snprintf(ld_library_path_buf, sizeof(ld_library_path_buf), "/opt/ros/%s/lib",
           getenv("ROS_DISTRO"));
  setenv("LD_LIBRARY_PATH", ld_library_path_buf, 1);

  // Init ROS node, delegating signal handling to rtapi_app
  auto init_opts = rclcpp::InitOptions();
  init_opts.shutdown_on_sigint = false;
  int argc;
  for (argc = 0; ARGV[argc] != nullptr; argc++)
  {
    HAL_ROS_DBG_NAMED(CNAME, "  ROS args:  %d = '%s'", argc, ARGV[argc]);
  }
  rclcpp::init(argc, ARGV, init_opts);

  // Init controller manager and asynch executor
  // - Resource manager & hardware interface loaded here; HAL pins initialized
  EXECUTOR.reset(new rclcpp::executors::MultiThreadedExecutor());
  std::string manager_node_name = "controller_manager";
  CONTROLLER_MANAGER.reset(
      new controller_manager::ControllerManager(EXECUTOR, manager_node_name));
  EXECUTOR->add_node(CONTROLLER_MANAGER);

  // ROS asynch executor thread pointer
  auto executor_cb = []() { EXECUTOR->spin(); };
  auto executor_thread = new std::thread(executor_cb);
  pthread_setname_np(executor_thread->native_handle(), "ros2_ctl_mgr");

  // Export the function & mark component ready
  if (hal_export_functf(funct, nullptr, 1, 0, comp_id, "%s.funct", CNAME) < 0)
  {
    HAL_ROS_INFO_NAMED(CNAME, "ERROR: hal_export_functf failed");
    hal_exit(comp_id);
    return -1;
  }
  hal_ready(comp_id);

  HAL_ROS_INFO_NAMED(CNAME, "%s:  HAL component ready!", CNAME);

  rclcpp::Logger l = CONTROLLER_MANAGER->get_logger();
  HAL_ROS_INFO(l, "HAL controller manager initializing");

  return 0;  // Success
}

void funct([[maybe_unused]] void* arg, [[maybe_unused]] int64_t period)
{
  CONTROLLER_MANAGER->read();
  CONTROLLER_MANAGER->update();
  CONTROLLER_MANAGER->write();
}

void rtapi_app_exit(void)
{
  rclcpp::Logger l = CONTROLLER_MANAGER->get_logger();
  HAL_ROS_INFO(l, "HAL controller manager shutting down");
  EXECUTOR->cancel();
  HAL_ROS_INFO(l, "Executor canceled");
  rclcpp::shutdown();
}

}  // extern "C"
