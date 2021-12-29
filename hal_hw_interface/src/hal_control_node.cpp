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
#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>
#include <hal_hw_interface/hal_ros_logging.hpp>
#include <hal_hw_interface/hal_def.hpp>
#include <memory>
#include <string>

// Controller manager pointer
std::shared_ptr<controller_manager::ControllerManager> CONTROLLER_MANAGER;

// ROS asynch executor thread pointer
std::shared_ptr<std::thread> EXECUTOR_THREAD;

extern "C" {
// Pre-declare the HAL function
void funct(void* arg, int64_t period);

#define MAX_ARGS 255
static char *argv[MAX_ARGS] = {0,};
RTAPI_MP_ARRAY_STRING(argv, MAX_ARGS, "ROS command-line argv");

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

  HAL_ROS_INFO_NAMED(CNAME, "%s: Initialized HAL component", CNAME);

  // Apparently rcl searches through $LD_LIBRARY_PATH to find
  // librmw_fastrtps_cpp.so and related libs.  :P Since HAL's rtapi_app runs
  // setuid, $LD_LIBRARY_PATH is ignored, so we have to artificially reconstruct
  // it.  Can't go through the ROS params, since no node yet, so fake it with
  // the single /opt/ros/$ROS_DISTRO/lib entry.
  char LD_LIBRARY_PATH_buf[256];
  snprintf(LD_LIBRARY_PATH_buf, sizeof(LD_LIBRARY_PATH_buf), "/opt/ros/%s/lib",
           getenv("ROS_DISTRO"));
  setenv("LD_LIBRARY_PATH", LD_LIBRARY_PATH_buf, 1);

  // Init ROS node, delegating signal handling to rtapi_app
  auto init_opts = rclcpp::InitOptions();
  init_opts.shutdown_on_sigint = false;
  // 'hal_control_node', '--ros-args', '--params-file', '/tmp/launch_params_9hqq2one', '--params-file', '/home/zultron/git/picknik/hatci_ws/install/hal_hw_interface/share/hal_hw_interface/config/hal_hw_interface.yaml', '--params-file', '/home/zultron/git/picknik/hatci_ws/install/ros2_control_demo_bringup/share/ros2_control_demo_bringup/config/rrbot_controllers.yaml']
  int argc;
  for (argc=0; argv[argc] != nullptr; argc++) {
    HAL_ROS_DBG_NAMED(CNAME, "  ROS args:  %d = '%s'", argc, argv[argc]);
  }
  rclcpp::init(argc, argv, init_opts);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("hal_control_node");

  // Init controller manager and asynch executor
  // - Resource manager & hardware interface loaded here; HAL pins initialized
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  std::string manager_node_name = "controller_manager";
  CONTROLLER_MANAGER.reset(
      new controller_manager::ControllerManager(executor, manager_node_name));

  // Launch executor in asynch thread
  EXECUTOR_THREAD.reset(new std::thread([&executor]() { executor->spin(); }));

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
  EXECUTOR_THREAD->join();
}

}  // extern "C"
