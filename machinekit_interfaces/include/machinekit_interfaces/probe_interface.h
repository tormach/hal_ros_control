#ifndef PROBE_INTERFACE_H
#define PROBE_INTERFACE_H

/// \author: Robert W. Ellenberg

#pragma once


#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace machinekit_interfaces
{

enum class ProbeState : int {
    DISCONNECTED=-1,
    OFF,
    ON,
    OVERTRAVEL,
};

enum class ProbeTransitions : int {
    INVALID=-1,
    NONE,
    RISING,
    FALLING,
};

/**
 * Communicates status of a probe in realtime from the control loop to any listening controllers.
 *
 * Commands:
 * Controllers can specify the condition for capture of a probe position.
 *
 * Feedback:
 * Probe signal state
 * Probe signal transitions detected
 *
 * This allows the controller to react in realtime to probing signals.
 */
class ProbeHandle
{
public:
  ProbeHandle() = default;

  ProbeHandle(const std::string& name, //!< Name of the probe
                     int* probe_capture_ptr, //!< Capture setting (i.e. rising means expect a rising edge, none implies we expect the probe signal to be off and no edges to be seen)
                     const int* probe_state_ptr, //!< Probe state signal
                     int* probe_transition_ptr, //!< transitions detected in the probe signal (handshake)
                     int* probe_result_type_ptr, //!< Kind of transition captured for the result
                     ros::Time* capture_time_ptr
              )
    : name_(name),
      probe_capture_(probe_capture_ptr),
      probe_state_(probe_state_ptr),
      probe_transition_(probe_transition_ptr),
      probe_result_type_(probe_result_type_ptr),
      capture_time_(capture_time_ptr)
  {}

  std::string getName()     const {return name_;}
  int getProbeState() const {
      return probe_state_ ? *probe_state_ : (int)ProbeState::DISCONNECTED;
  }
  int acquireProbeTransition() {
      if (!probe_transition_) {
          return (int)ProbeTransitions::INVALID;
      }
      int transition = *probe_transition_;
      *probe_transition_ = 0;
      return transition;
  }

  int getProbeCapture() const {
      return probe_capture_ ? *probe_capture_ : (int)ProbeTransitions::INVALID;
  }

  int getProbeResultType() const {
      return probe_result_type_ ? *probe_result_type_ : (int)ProbeTransitions::INVALID;
  }

  ros::Time getProbeCaptureTime() const {
      return capture_time_ ? *capture_time_ : (ros::Time)0;
  }

  /** Set from controller to tell the hardware when to capture joint state */
  void setProbeCapture(int to_capture) {
      assert(probe_capture_);
      *probe_capture_ = to_capture;
      assert(probe_result_type_);
      *probe_result_type_ = 0;
  }

private:
  std::string name_;
  int * probe_capture_  = {nullptr};
  const int * probe_state_  = {nullptr};
  int * probe_transition_  = {nullptr};
  int * probe_result_type_  = {nullptr};
  ros::Time * capture_time_ = {nullptr};
};


/** \brief Hardware interface to support reading the state of a force-torque sensor. */
class ProbeInterface : public hardware_interface::HardwareResourceManager<ProbeHandle,  hardware_interface::ClaimResources> {};

}

#endif // PROBE_INTERFACE_H
