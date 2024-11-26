// velocity_scale_base.cpp
#include "interruptible_joint_trajectory_controller/velocity_scale_base.h"

void VelocityScaleBase::update(double period)
{
    double target = rt_target_scaling_factor_.load();
    double current = rt_current_scaling_factor_.load();

    if (std::abs(target - current) < TRANSITION_TIME_INTERPOLATION_TRESH)
    {
        rt_current_scaling_factor_.store(target);
        return;
    }

    // Use fast deceleration rate if stop is active
    double decel_rate = is_stop_active_ ? STOP_DECEL_RATE : scaling_factor_increment_;

    // If stop is active, force target to zero
    if (is_stop_active_)
    {
        target = 0.0;
    }

    // Update current scaling factor
    if (target > current)
    {
        current = std::min(target, current + scaling_factor_increment_ * period / CONTROL_CYCLE_TIME);
    }
    else
    {
        current = std::max(target, current - decel_rate * period / CONTROL_CYCLE_TIME);
        //current = std::max(target, current - 0.01);
        //if(current < 0.0) current = 0.0;
    }

    rt_current_scaling_factor_.store(current);
}

void VelocityScaleBase::updateTargetScalingFactor(double target_factor)
{
    if (!is_stop_active_)  // Only update target if stop is not active
    {
        rt_target_scaling_factor_.store(std::max(0.0, std::min(1.0, target_factor)));
    }
}

void VelocityScaleBase::updateTransitionTime(double transition_time)
{
    if (transition_time > TRANSITION_TIME_INTERPOLATION_TRESH)
    {
        rt_transition_time_.store(transition_time);
        scaling_factor_increment_ = CONTROL_CYCLE_TIME / transition_time;
    }
}

void VelocityScaleBase::triggerStop()
{
    is_stop_active_ = true;
    rt_target_scaling_factor_.store(0.0);
}

void VelocityScaleBase::clearStop()
{
    is_stop_active_ = false;
}
