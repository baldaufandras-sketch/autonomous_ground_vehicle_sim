#pragma once

#include <vector>

#include "agv_sim/path_geometry.hpp"
#include "agv_sim/vehicle.hpp"
#include "agv_sim/waypoint.hpp"

struct PurePursuitConfig {
  double lookahead_distance;
  double wheelbase;
  double fallback_steering_angle;
};

struct ControllerDebugInfo {
  std::size_t current_waypoint_index{};
  double target_x{};
  double target_y{};
  double pursuit_controller_alpha{};
  double steering_angle_request{};
};

class PurePursuitController {
public:
  PurePursuitController(const PurePursuitConfig &config);

  VehicleInput computeControl(const VehicleState &state,
                              const std::vector<Waypoint> &waypoints);

  std::size_t getCurrentWaypointIndex() const;
  void setLookaheadDistance(double lookAhead);

  void reset(std::size_t start_index);

  Waypoint findLookaheadPoint(const VehicleState &state,
                              const std::vector<Waypoint> &waypoints) const;
  ControllerDebugInfo getControllerDebugInfo() const;

private:
  double computeSteeringAngle(const VehicleState &state,
                              const Waypoint &target_point);

  void updateCurrentWaypoint(const VehicleState &state,
                             const std::vector<Waypoint> &waypoints);

  double computeAcceleration(const std::vector<Waypoint> &waypoints) const;

  PurePursuitConfig config_{};
  std::size_t current_waypoint_index_{0};
  ControllerDebugInfo debug_info_;
};