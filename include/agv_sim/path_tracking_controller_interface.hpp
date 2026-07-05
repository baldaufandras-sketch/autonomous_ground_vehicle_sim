#pragma once

#include "agv_sim/controller_debug_info.hpp"
#include "agv_sim/path.hpp"
#include "agv_sim/vehicle.hpp"
#include <vector>

class IPathTrackingController {
public:
  virtual ~IPathTrackingController() = default;

  virtual VehicleInput computeControl(const VehicleState &state,
                                      const Path &path) = 0;

  virtual ControllerDebugInfo getControllerDebugInfo() const = 0;
};