#ifndef VELOCITY_SCALE_BASE_H
#define VELOCITY_SCALE_BASE_H

#include <atomic>
#include <cmath>
#include <algorithm>

class VelocityScaleBase
{
public:
    static constexpr double TRANSITION_TIME_INTERPOLATION_TRESH = 0.010;
    static constexpr double DEFAULT_TRANSITION_TIME = 0.2;  // in seconds
    static constexpr double CONTROL_CYCLE_TIME = 0.001;     // in seconds
    static constexpr double STOP_DECEL_TIME = 0.05;        // 50ms for emergency stop
    static constexpr double NORMAL_TRANSITION_RATE = CONTROL_CYCLE_TIME / DEFAULT_TRANSITION_TIME;
    static constexpr double STOP_DECEL_RATE = CONTROL_CYCLE_TIME / STOP_DECEL_TIME;  // Much faster deceleration for stop

    VelocityScaleBase()
    {
        rt_transition_time_.store(DEFAULT_TRANSITION_TIME);
        scaling_factor_increment_ = NORMAL_TRANSITION_RATE;
        is_stop_active_ = false;
    }

    void update(double period);
    void updateTargetScalingFactor(double target_factor);
    void updateTransitionTime(double transition_time);
    void triggerStop();  // New method to activate emergency stop
    void clearStop();    // New method to clear stop state

    double getCurrentScalingFactor() const
    {
        return rt_current_scaling_factor_.load();
    }

    double getTargetScalingFactor() const
    {
        return rt_target_scaling_factor_.load();
    }

protected:
    std::atomic<double> rt_target_scaling_factor_{ 1.0 };
    std::atomic<double> rt_current_scaling_factor_{ 1.0 };
    std::atomic<double> rt_transition_time_;
    std::atomic<bool> is_stop_active_{ false };
    double scaling_factor_increment_;
};

#endif // VELOCITY_SCALE_BASE_H
