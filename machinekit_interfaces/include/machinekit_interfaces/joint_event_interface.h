#ifndef JOINT_EVENT_DATA_INTERFACE_H
#define JOINT_EVENT_DATA_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <string>

namespace machinekit_interfaces
{

// Thin wrappers around joint state just to avoid confusing the rest of the system (normal joint handles represent actual joints in the URDF)

class JointEventDataHandle : public hardware_interface::JointStateHandle {
public:
  JointEventDataHandle() = default;
  JointEventDataHandle(const std::string& name, const double* pos, const double* vel, const double* eff) : JointStateHandle(name, pos, vel, eff) {}
};
class JointEventDataInterface : public hardware_interface::HardwareResourceManager<JointEventDataHandle, hardware_interface::ClaimResources> {};

}
#endif // JOINT_EVENT_DATA_INTERFACE_H
