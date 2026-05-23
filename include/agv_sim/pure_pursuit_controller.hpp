#pragma once

#include <vector>

#include "agv_sim/vehicle.hpp"
#include "agv_sim/waypoint.hpp"

struct Point2D {
  double x{};
  double y{};
};

class PurePursuitController {
public:
  PurePursuitController(double lookahead_distance, double wheelbase);

  VehicleInput computeControl(const VehicleState &state,
                              const std::vector<Waypoint> &waypoints) const;

private:
  Point2D findLookaheadPoint(const VehicleState &state,
                             const std::vector<Waypoint> &waypoints) const;

  double computeSteeringAngle(const VehicleState &state,
                              const Point2D &target_point) const;

  double lookahead_distance_{};
  double wheelbase_{};
};