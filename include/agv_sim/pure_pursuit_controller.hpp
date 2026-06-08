#pragma once

#include <vector>

#include "agv_sim/path_geometry.hpp"
#include "agv_sim/vehicle.hpp"
#include "agv_sim/waypoint.hpp"

class PurePursuitController {
public:
  PurePursuitController(double lookahead_distance, double wheelbase);

  VehicleInput computeControl(const VehicleState &state,
                              const std::vector<Waypoint> &waypoints);

  std::size_t getCurrentWaypointIndex() const;
  void setLookaheadDistance(double lookAhead);

  void reset(std::size_t start_index);

  Waypoint findLookaheadPoint(const VehicleState &state,
                              const std::vector<Waypoint> &waypoints) const;

private:
  double computeSteeringAngle(const VehicleState &state,
                              const Waypoint &target_point) const;

  void updateCurrentWaypoint(const VehicleState &state,
                             const std::vector<Waypoint> &waypoints);

  double lookahead_distance_{};
  double wheelbase_{};
  std::size_t current_waypoint_index_{0};
};