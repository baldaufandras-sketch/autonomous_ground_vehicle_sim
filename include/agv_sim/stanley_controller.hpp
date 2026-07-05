#pragma once

#include "agv_sim/controller_debug_info.hpp"
#include "agv_sim/path.hpp"
#include "agv_sim/path_geometry.hpp"
#include "agv_sim/vehicle.hpp"
#include <algorithm>
#include <cstdlib>
#include <limits>
#include <vector>

struct StanleyConfig {
  double stanley_gain;
};

class StanleyController {
public:
  StanleyController(const StanleyConfig &config);

  void updateCurrentSegmentIndex(const Path &path, const VehicleState &state);

  VehicleInput computeControl(const VehicleState &state, const Path &path);

  ControllerDebugInfo getControllerDebugInfo() const;

private:
  ControllerDebugInfo debug_info_;
  StanleyConfig config_{};
  std::size_t current_segment_index_{0};
  double calculateSignedLateralError(const VehicleState &state,
                                     const Waypoint &closest_point,
                                     const PathSegment &segment);
};
