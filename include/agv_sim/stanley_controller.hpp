#pragma once

#include "agv_sim/controller_debug_info.hpp"
#include "agv_sim/path.hpp"
#include "agv_sim/path_geometry.hpp"
#include "agv_sim/vehicle.hpp"
#include <cstdlib>
#include <vector>

struct StanleyConfig {
  double stanley_gain;
};

class StanleyController {
public:
  StanleyController(const StanleyConfig &config);

  Waypoint closestPointToSegment(const PathSegment &segment,
                                 const VehicleState &state);

  PathSegment findClosestSegment(const Path &path, const VehicleState &state);

  VehicleInput computeControl(const VehicleState &state, const Path &path);

  ControllerDebugInfo getControllerDebugInfo() const;

private:
  ControllerDebugInfo debug_info_;
  StanleyConfig config_{};
};