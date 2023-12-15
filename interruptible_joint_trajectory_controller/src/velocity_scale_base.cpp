#include "interruptible_joint_trajectory_controller/velocity_scale_base.h"

void VelocityScaleBase::updateTransitionTime(double transition_time)
{
  rt_transition_time_.store(transition_time);
  scaling_factor_increment_ = CONTROL_CYCLE_TIME / transition_time;
}

void VelocityScaleBase::updateTargetScalingFactor(double target_factor)
{
  rt_target_scaling_factor_.store(target_factor);
}

void VelocityScaleBase::update(double period)
{
  // TODO:ERROR when uncommenting
  // ROS_WARN("SCALE_UPDATE");
  double target_scaling_factor = rt_target_scaling_factor_.load();
  double full_transition_time = rt_transition_time_.load();

  scaling_factor_increment_ = period / full_transition_time;

  if (full_transition_time < TRANSITION_TIME_INTERPOLATION_TRESH)
  {
    rt_current_scaling_factor_.store(target_scaling_factor);
  }
  else if (std::abs(target_scaling_factor - rt_current_scaling_factor_) <=
           scaling_factor_increment_)
  {
    rt_current_scaling_factor_.store(target_scaling_factor);
  }
  else
  {
    double no_rt_current_scaling_factor = rt_current_scaling_factor_.load();
    if (target_scaling_factor > rt_current_scaling_factor_)
    {
      no_rt_current_scaling_factor += scaling_factor_increment_;
    }
    else if (target_scaling_factor < rt_current_scaling_factor_)
    {
      no_rt_current_scaling_factor -= scaling_factor_increment_;
    }
    rt_current_scaling_factor_.store(no_rt_current_scaling_factor);
  }
}
