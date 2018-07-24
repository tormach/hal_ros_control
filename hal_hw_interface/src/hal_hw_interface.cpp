#include <hal_hw_interface/hal_hw_interface.h>
#include <hal_hw_interface/hal_ros_logging.h>


namespace hal_hw_interface
{

HalHWInterface::HalHWInterface(
  ros::NodeHandle &nh, urdf::Model *urdf_model)
: ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
}

void HalHWInterface::init(void (*funct)(void*, long))
{
  HAL_ROS_LOG_INFO(CNAME, "%s: Initializing HAL hardware interface", CNAME);

  // Call boilerplate init() function
  ros_control_boilerplate::GenericHWInterface::init();

  HAL_ROS_LOG_INFO(CNAME, "%s: Initialized boilerplate", CNAME);

  // Initialize component
  comp_id_ = hal_init(CNAME);
  if (comp_id_ < 0)
  {
    HAL_ROS_LOG_ERR(CNAME, "%s:  ERROR: Component creation ABORTED", CNAME);
    //return false; // FIXME
    return;
  }

  HAL_ROS_LOG_INFO(CNAME, "%s: Initialized HAL component", CNAME);

  // Initialize HAL pins for each joint
  for (std::size_t ix = 0; ix < num_joints_; ix++)
  {
    // init_joint(ix);
    HAL_ROS_LOG_INFO(
      CNAME, "%s: Init joint #%zu %s", CNAME, ix, joint_names_[ix].c_str());

    if (!create_float_pin(ix, &joint_pos_cmd_ptrs_, HAL_OUT, "pos-cmd") ||
        !create_float_pin(ix, &joint_vel_cmd_ptrs_, HAL_OUT, "vel-cmd") ||
        !create_float_pin(ix, &joint_eff_cmd_ptrs_, HAL_OUT, "eff-cmd") ||
        !create_float_pin(ix, &joint_pos_fb_ptrs_, HAL_IN, "pos-fb") ||
        !create_float_pin(ix, &joint_vel_fb_ptrs_, HAL_IN, "vel-fb") ||
        !create_float_pin(ix, &joint_eff_fb_ptrs_, HAL_IN, "eff-fb"))
    {
      HAL_ROS_LOG_ERR(
        CNAME, "%s: Failed to initialize joint %zu %s.%s",
        CNAME, ix, CNAME, joint_names_[ix].c_str());
      //return false; // FIXME
      return;
    }
  }
  HAL_ROS_LOG_INFO(CNAME, "%s:  Initialized HAL pins", CNAME);

  // Export the function
  if (hal_export_functf(funct, this, 1, 0, comp_id_, "%s.funct", CNAME) < 0)
  {
    HAL_ROS_LOG_INFO(CNAME, "%s: ERROR: hal_export_functf failed", CNAME);
    hal_exit(comp_id_);
    //return false; // FIXME
    return;
  }
  HAL_ROS_LOG_INFO(CNAME, "%s:  Exported HAL function", CNAME);

  // Mark component ready
  hal_ready(comp_id_);

  HAL_ROS_LOG_INFO(CNAME, "%s:  HAL component ready!", CNAME);

  //return true; // FIXME
} // init()

bool HalHWInterface::create_float_pin(
  const std::size_t ix, std::vector<double**> *ptrs,
  hal_pin_dir_t dir, const char* name)
{
  // Sanity check vector length
  if (ptrs->size() != ix)
  {
    HAL_ROS_LOG_ERR(
      CNAME, "%s: Size of pin storage not consistent with ID", CNAME);
    return false;
  }
  // Allocate space
  ptrs->push_back((hal_float_t**) hal_malloc(sizeof(hal_float_t*)));
  if (ptrs->at(ix) == NULL) {
    HAL_ROS_LOG_ERR(CNAME, "%s: Allocate HAL pin failed", CNAME);
    return false;
  }
  if (hal_pin_float_newf(dir, ptrs->at(ix), comp_id_,
                         "%s.%s.%s", CNAME, joint_names_[ix].c_str(), name))
  {
    HAL_ROS_LOG_INFO(
      CNAME, "%s: New HAL pin %s.%s.%s failed",
      CNAME, CNAME, joint_names_[ix].c_str(), name);
    return false;
  }

  HAL_ROS_LOG_INFO(
    CNAME, "%s: New HAL pin %s.%s.%s succeeded; addr %p",
    CNAME, CNAME, joint_names_[ix].c_str(), name, ptrs->at(ix));
  return true;
}


void HalHWInterface::read(ros::Duration &elapsed_time)
{
  // Copy HAL joint feedback pin values to controller joint states
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    joint_position_[joint_id] = **joint_pos_fb_ptrs_[joint_id];
    joint_velocity_[joint_id] = **joint_vel_fb_ptrs_[joint_id];
    joint_effort_[joint_id]   = **joint_eff_fb_ptrs_[joint_id];
  }
}

void HalHWInterface::write(ros::Duration &elapsed_time)
{
  // Enforce joint limits
  enforceLimits(elapsed_time);
  // Copy controller joint command values to HAL joint command pins
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    **joint_pos_cmd_ptrs_[joint_id] = joint_position_command_[joint_id];
    **joint_vel_cmd_ptrs_[joint_id] = joint_velocity_command_[joint_id];
    **joint_eff_cmd_ptrs_[joint_id] = joint_effort_command_[joint_id];
  }
}


void HalHWInterface::enforceLimits(ros::Duration &period)
{
  // FIXME how does this fit in?  Should it just be done in HAL?
  // from sim_hw_interface.cpp:
  //pos_jnt_sat_interface_.enforceLimits(period);
}

void HalHWInterface::shutdown()
{
  if (! comp_id_)
  {
    HAL_ROS_LOG_ERR(CNAME, "%s: HAL already shut down", CNAME);
  } else {
    HAL_ROS_LOG_INFO(CNAME, "%s: HAL shutting down", CNAME);
    hal_exit(comp_id_);
    comp_id_ = 0;
  }
}

} // namespace
