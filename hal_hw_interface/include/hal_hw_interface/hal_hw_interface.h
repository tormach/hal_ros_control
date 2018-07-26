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

#ifndef HAL_HW_INTERFACE_HAL_HW_INTERFACE_H
#define HAL_HW_INTERFACE_HAL_HW_INTERFACE_H

// hal_hw_interface subclasses ros_control_boilerplate
#include <ros_control_boilerplate/generic_hw_interface.h>

// HAL
#include <hal.h>

// ROS
#include <ros/duration.h>

// Component name
#define CNAME "hal_hw_interface"

namespace hal_hw_interface
{

  class HalHWInterface : public ros_control_boilerplate::GenericHWInterface
  {
  public:

    /**
     * \brief Constructor
     * \param nh - ROS node handle
     * \param urdf_model - optional pointer to a parsed robot model
     */

    HalHWInterface(
      ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);

    /**
     * \brief Initialize the hardware interface
     * \param funct - the component function run periodically in a HAL RT thread
     */
    void init(void (*funct)(void*, long));

    /**
     * \brief Create float-type HAL pin
     * \param ix - pin
     * \param ptrs - a vector of double pointers to HAL float storage
     * \param dir - pin direction; one of HAL_IN, HAL_OUT
     * \param func - the HAL component function that runs read/update/write
     */
    bool create_float_pin(
      const std::size_t ix,
      std::vector<double**> *ptrs,
      hal_pin_dir_t dir,
      const char* func);

    /**
     * \brief Read the state from the robot hardware.
     * \param elapsed_time - period since last run
     */
    void read(ros::Duration &elapsed_time);

    /**
     * \brief Write the command to the robot hardware.
     * \param elapsed_time - period since last run
     */
    void write(ros::Duration &elapsed_time);
  
    /**
     * \brief Enforce joint limits
     * \param elapsed_time - period since last run
     */
    void enforceLimits(ros::Duration &period);

    /**
     * \brief Shut down the HAL component and the ROS node
     */
    void shutdown();

  protected:

    /**
     * \brief HAL component ID
     */
    int comp_id_;

  private:

    // Joints:  HAL storage
    // - Commands
    std::vector<double**> joint_pos_cmd_ptrs_;
    std::vector<double**> joint_vel_cmd_ptrs_;
    std::vector<double**> joint_eff_cmd_ptrs_;
    // - States
    std::vector<double**> joint_pos_fb_ptrs_;
    std::vector<double**> joint_vel_fb_ptrs_;
    std::vector<double**> joint_eff_fb_ptrs_;


  }; // HalHWInterface

} // hardware_interface

#endif // HAL_HW_INTERFACE_HAL_HW_INTERFACE_H
