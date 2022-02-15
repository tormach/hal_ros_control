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
#include <unistd.h>   // sleep()
#include <hal.h>      // HAL public API decls
#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/duration.hpp>
#include <hardware_interface/macros.hpp>  // THROW_ON_NULLPTR
#include <lifecycle_msgs/msg/state.hpp>
#include <hal_hw_interface/hal_ros_logging.hpp>
#include <memory>
#include <string>
#include <vector>

// Controller manager node and executor pointers
std::shared_ptr<controller_manager::ControllerManager> CONTROLLER_MANAGER;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> EXECUTOR;
std::shared_ptr<std::thread> EXECUTOR_THREAD;
rclcpp::TimerBase::SharedPtr RESET_TIMER;
hal_bit_t** RESET_PIN_PTR;
hal_bit_t** CM_OK_PIN_PTR;
bool CM_OK = 1;
int COMP_ID = -1;

extern "C" {
// Pre-declare the HAL function
void funct(void* arg, int64_t period);

#define MAX_ARGS 255
static char* ARGV[MAX_ARGS] = {
  nullptr,
};
RTAPI_MP_ARRAY_STRING(ARGV, MAX_ARGS, "ROS command-line argv");

// Reset HAL pin callback
//
// A high `reset` pin value means we want command reset to feedback.  Controller
// `on_activate()` function should do so.
//
// Called periodically from non-RT thread timer, when `reset` pin is set,
// request each active controller be deactivated & activated (and clear `reset`
// pin).
void reset_controller_cb(void)
{
  THROW_ON_NULLPTR(RESET_PIN_PTR);
  THROW_ON_NULLPTR(*RESET_PIN_PTR);
  if (!**RESET_PIN_PTR)
    return;

  uint64_t period = 10 * 1000 * 1000;  // 10ms

  // reset pin went high; reset command to feedback by stopping and starting
  // all active controllers during update()
  HAL_ROS_INFO_NAMED(CNAME, "Reset pin high; reactivating controllers:");
  **RESET_PIN_PTR = 0;  // clear reset pin
  std::vector<std::string> start_controllers, stop_controllers;
  for (const auto& controller : CONTROLLER_MANAGER->get_loaded_controllers())
    if (controller.c->get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      HAL_ROS_INFO_NAMED(CNAME, "  - %s", controller.info.name.c_str());
      start_controllers.push_back(controller.info.name);
      stop_controllers.push_back(controller.info.name);
    }
  CONTROLLER_MANAGER->switch_controller(
      start_controllers, stop_controllers,
      controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT,
      true,  // start_asap
      rclcpp::Duration((int32_t)(period >> 32),
                       (uint32_t)(period))  // timeout
  );
  HAL_ROS_INFO_NAMED(CNAME, "  ...complete");
}

int rtapi_app_main(void)
{
  // Init HAL component
  COMP_ID = hal_init(CNAME);
  if (COMP_ID < 0)
  {
    HAL_ROS_ERR_NAMED(CNAME, "ERROR: Component '%s' creation ABORTED", CNAME);
    // return false; // FIXME
    return -1;
  }

  HAL_ROS_INFO_NAMED(CNAME, "Initialized HAL component '%s' ID %d", CNAME,
                     COMP_ID);

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

  // Set up reset HAL pin
  HAL_ROS_DBG_NAMED(CNAME, "Initializing %s.reset HAL pin", CNAME);
  RESET_PIN_PTR = reinterpret_cast<hal_bit_t**>(hal_malloc(sizeof(hal_bit_t*)));
  THROW_ON_NULLPTR(RESET_PIN_PTR);
  if (hal_pin_bit_newf(HAL_IO, RESET_PIN_PTR, COMP_ID, "%s.reset", CNAME))
    throw std::runtime_error("Failed to init " + std::string(CNAME) +
                             ".reset HAL pin");
  THROW_ON_NULLPTR(*RESET_PIN_PTR);

  // Set up cm_ok pin
  HAL_ROS_DBG_NAMED(CNAME, "Initializing %s.cm_ok HAL pin", CNAME);
  CM_OK_PIN_PTR = reinterpret_cast<hal_bit_t**>(hal_malloc(sizeof(hal_bit_t*)));
  if (hal_pin_bit_newf(HAL_OUT, CM_OK_PIN_PTR, COMP_ID, "%s.cm_ok", CNAME))
    throw std::runtime_error("Failed to init " + std::string(CNAME) +
                             ".cm_ok HAL pin");
  THROW_ON_NULLPTR(*CM_OK_PIN_PTR);

  // Init controller manager and asynch executor
  HAL_ROS_DBG_NAMED(CNAME, "Initializing node executor");
  // - Resource manager & hardware interface loaded here; HAL pins initialized
  EXECUTOR.reset(new rclcpp::executors::MultiThreadedExecutor());
  std::string manager_node_name = "controller_manager";
  CONTROLLER_MANAGER.reset(
      new controller_manager::ControllerManager(EXECUTOR, manager_node_name));
  // - Timer for reactivating controllers when reset pin set
  RESET_TIMER = CONTROLLER_MANAGER->create_wall_timer(
      std::chrono::milliseconds(10), reset_controller_cb);
  EXECUTOR->add_node(CONTROLLER_MANAGER);

  // ROS asynch executor thread pointer
  HAL_ROS_DBG_NAMED(CNAME, "Starting executor");
  auto executor_cb = []() { EXECUTOR->spin(); };
  EXECUTOR_THREAD.reset(new std::thread(executor_cb));
  pthread_setname_np(EXECUTOR_THREAD->native_handle(), "ros2_ctl_mgr");

  // Export the function & mark component ready
  HAL_ROS_DBG_NAMED(CNAME, "Exporting HAL function; marking component ready");
  if (hal_export_functf(funct, nullptr, 1, 0, COMP_ID, "%s.funct", CNAME) < 0)
  {
    HAL_ROS_INFO_NAMED(CNAME, "ERROR: hal_export_functf failed");
    hal_exit(COMP_ID);
    return -1;
  }
  hal_ready(COMP_ID);
  HAL_ROS_INFO_NAMED(CNAME, "HAL component ready!");

  // Log first marker message with controller manager logger
  auto l = CONTROLLER_MANAGER->get_logger();
  HAL_ROS_INFO(l, "HAL controller manager initializing");

  return 0;  // Success
}

void funct([[maybe_unused]] void* arg, [[maybe_unused]] int64_t period)
{
  if (!rclcpp::ok())
    CM_OK = 0;
  **CM_OK_PIN_PTR = CM_OK;
  if (!CM_OK)
    return;
  CONTROLLER_MANAGER->read();
  CONTROLLER_MANAGER->update();
  CONTROLLER_MANAGER->write();
}

void rtapi_app_exit(void)
{
  auto l = CONTROLLER_MANAGER->get_logger();
  HAL_ROS_INFO(l, "rtapi_app_exit():  HAL controller manager shutting down");
  CM_OK = 0;  // In case unloadrt before threads stopped
  RESET_TIMER->cancel();
  HAL_ROS_DBG(l, "Reset timer canceled");
  EXECUTOR->remove_node(CONTROLLER_MANAGER);
  HAL_ROS_DBG(l, "Controller manager node removed from executor");
  EXECUTOR->cancel();
  HAL_ROS_DBG(l, "Executor canceled");
  EXECUTOR_THREAD->join();
  HAL_ROS_DBG(l, "Executor thread joined");

  RESET_TIMER.reset();
  EXECUTOR.reset();
  CONTROLLER_MANAGER.reset();
  EXECUTOR_THREAD.reset();
  HAL_ROS_INFO(l, "HAL controller manager Node shut down");

  hal_exit(COMP_ID);
  HAL_ROS_INFO(l, "HAL comp exited");
}

}  // extern "C"
