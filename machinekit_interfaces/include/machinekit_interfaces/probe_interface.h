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
                     const int* probe_transition_ptr //!< transitions detected in the probe signal
              )
    : name_(name),
      probe_capture_(probe_capture_ptr),
      probe_state_(probe_state_ptr),
      probe_transition_(probe_transition_ptr)
  {}

  std::string getName()     const {return name_;}
  int getProbeState() const {
      return probe_state_ ? *probe_state_ : (int)ProbeState::DISCONNECTED;
  }
  int getProbeTransition() const {
      return probe_transition_ ? *probe_transition_ : (int)ProbeTransitions::INVALID;
  }
  int getProbeCapture() const {
      return probe_capture_ ? *probe_capture_ : (int)ProbeTransitions::INVALID;
  }

  /** Set from controller to tell the hardware when to capture joint state */
  void setProbeCapture(int to_capture) {
      assert(probe_capture_);
      *probe_capture_ = to_capture;
  }

private:
  std::string name_;
  int * probe_capture_  = {nullptr};
  const int * probe_state_  = {nullptr};
  const int * probe_transition_  = {nullptr};
};


/** \brief Hardware interface to support reading the state of a force-torque sensor. */
class ProbeInterface : public hardware_interface::HardwareResourceManager<ProbeHandle> {};

}

#endif // PROBE_INTERFACE_H
