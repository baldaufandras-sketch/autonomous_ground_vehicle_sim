#include "agv_sim/pure_pursuit_controller.hpp"

PurePursuitController::PurePursuitController(double lookahead_distance,
                                             double wheelbase)
    : lookahead_distance_{lookahead_distance}, wheelbase_{wheelbase} {}

VehicleInput PurePursuitController::computeControl(
    const VehicleState &state, const std::vector<Waypoint> &waypoints) const {
  Point2D target_point = findLookaheadPoint(state, waypoints);

  double steering_angle = computeSteeringAngle(state, target_point);

  return VehicleInput{.acceleration = 0, .steering_angle = steering_angle};
}

Point2D PurePursuitController::findLookaheadPoint(
    const VehicleState &state, const std::vector<Waypoint> &waypoints) const {
  return Point2D{};
}

double
PurePursuitController::computeSteeringAngle(const VehicleState &state,
                                            const Point2D &target_point) const {
  return 0.0;
}
