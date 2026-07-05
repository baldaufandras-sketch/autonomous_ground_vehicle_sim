#pragma once

#include "agv_sim/controller_debug_info.hpp"
#include "agv_sim/path.hpp"
#include "agv_sim/path_geometry.hpp"
#include "agv_sim/path_tracking_controller_interface.hpp"
#include "agv_sim/vehicle.hpp"
#include <vector>

struct PurePursuitConfig {
  double lookahead_distance;
  double wheelbase;
  double fallback_steering_angle;
};

struct SteeringCalculationResult {
  double steering_angle_request{};
  double alpha{};
  double target_heading{};
};

class PurePursuitController : public IPathTrackingController {
public:
  PurePursuitController(const PurePursuitConfig &config);

  VehicleInput computeControl(const VehicleState &state,
                              const Path &path) override;

  std::size_t getCurrentWaypointIndex() const;
  void setLookaheadDistance(double lookAhead);

  void reset(std::size_t start_index);

  Waypoint findLookaheadPoint(const VehicleState &state,
                              const std::vector<Waypoint> &waypoints) const;
  ControllerDebugInfo getControllerDebugInfo() const override;

private:
  SteeringCalculationResult computeSteeringAngle(const VehicleState &state,
                                                 const Waypoint &target_point);

  void updateCurrentWaypoint(const VehicleState &state,
                             const std::vector<Waypoint> &waypoints);

  double computeAcceleration(const std::vector<Waypoint> &waypoints) const;

  PurePursuitConfig config_{};
  std::size_t current_waypoint_index_{0};
  ControllerDebugInfo debug_info_;
};