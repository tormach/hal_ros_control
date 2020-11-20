#ifndef REALTIME_EVENT_INTERFACE_H
#define REALTIME_EVENT_INTERFACE_H

/// \author: Robert W. Ellenberg

#pragma once


#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace machinekit_interfaces
{
enum class StopEventCodes : int {
    DISCONNECTED = -1,
    NONE,
    SAFETY_INPUT,
};

/**
 * Reports a realtime event code from the control loop to any listening controllers.
 * This way controllers can see events (as defined by the robot control loop, and if necessary react to them in realtime).
 */
class RealtimeEventHandle
{
public:
  RealtimeEventHandle() = default;

  RealtimeEventHandle(const std::string& name, //!< Name of the realtime event (e.g. stop event)
                     const long* event_code_ptr //!< pointer to data for the event code
                      ) : name_(name),
      event_code_(event_code_ptr)
  {}

  std::string getName() const
  {
      return name_;
  }
  long getEventCode() const {
      assert(event_code_);
      return *event_code_;
  }

private:
  std::string name_;
  const long * event_code_  = {nullptr};
};


/** \brief Hardware interface to support reading the state of a force-torque sensor. */
class RealtimeEventInterface : public hardware_interface::HardwareResourceManager<RealtimeEventHandle> {};

}
#endif // REALTIME_EVENT_INTERFACE_H
