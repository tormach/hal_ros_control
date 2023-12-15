#ifndef VELOCITY_SCALE_BASE_H
#define VELOCITY_SCALE_BASE_H

#include <atomic>
#include <cmath>  // for std::abs

class VelocityScaleBase
{
public:
  static constexpr double TRANSITION_TIME_INTERPOLATION_TRESH = 0.010;
  static constexpr double DEFAULT_TRANSITION_TIME = 1.0;  // in seconds
  static constexpr double CONTROL_CYCLE_TIME = 0.001;     // in seconds

  VelocityScaleBase()
  {
    rt_transition_time_.store(DEFAULT_TRANSITION_TIME);
    scaling_factor_increment_ = CONTROL_CYCLE_TIME / DEFAULT_TRANSITION_TIME;
  }

  void update(double period);  // Update scaling factor
  void updateTargetScalingFactor(double target_factor);  // Update scaling
                                                         // factor
  void updateTransitionTime(double transition_time);
  double getCurrentScalingFactor() const
  {
    return rt_current_scaling_factor_.load();
  }

protected:
  std::atomic<double> rt_target_scaling_factor_{ 1.0 };   // overwritten by
                                                          // derived class
  std::atomic<double> rt_current_scaling_factor_{ 1.0 };  // overwritten by
                                                          // derived class
  std::atomic<double> rt_transition_time_;
  double scaling_factor_increment_;
};

#endif  // VELOCITY_SCALE_BASE_H
