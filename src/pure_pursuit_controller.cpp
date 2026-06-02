#include "agv_sim/pure_pursuit_controller.hpp"
#include <iostream>

PurePursuitController::PurePursuitController(double lookahead_distance,
                                             double wheelbase)
    : lookahead_distance_{lookahead_distance}, wheelbase_{wheelbase} {}

VehicleInput
PurePursuitController::computeControl(const VehicleState &state,
                                      const std::vector<Waypoint> &waypoints) {
  // First current waypoint index shall be updated
  updateCurrentWaypoint(state, waypoints);

  // After the target waypoint is known, the lookahead point can be calculated
  Waypoint target_point = findLookaheadPoint(state, waypoints);

  // After the lookahead point is known, the steering angle can be calculated
  double steering_angle = computeSteeringAngle(state, target_point);

  return VehicleInput{.acceleration = 0, .steering_angle = steering_angle};
}

Waypoint PurePursuitController::findLookaheadPoint(
    const VehicleState &state, const std::vector<Waypoint> &waypoints) const {
  // TODO
  return Waypoint{};
}

void PurePursuitController::updateCurrentWaypoint(
    const VehicleState &state, const std::vector<Waypoint> &waypoints) {
  size_t targetIndex = PurePursuitController::current_waypoint_index_;
  size_t maxWaypoint = waypoints.size() - 1;
  double distanceToWaypoint{};
  double dx;
  double dy;
  for (size_t i = targetIndex; i < maxWaypoint; i++) {
    dx = state.x - waypoints[i].x;
    dy = state.y - waypoints[i].y;
    distanceToWaypoint = std::sqrt(dx * dx + dy * dy);
    // std::cout << distanceToWaypoint << std::endl;
    if (distanceToWaypoint >= lookahead_distance_) {
      break;
    }
    if (targetIndex == maxWaypoint) {
      current_waypoint_index_ = maxWaypoint;
      break;
    }
    current_waypoint_index_ += 1;
    // std::cout << current_waypoint_index_ << std::endl;
  }
}

double PurePursuitController::computeSteeringAngle(
    const VehicleState &state, const Waypoint &target_point) const {
  // TODO
  return (3.14 / 20);
}

void PurePursuitController::reset(std::size_t start_index = 0) {
  current_waypoint_index_ = start_index;
}

std::size_t PurePursuitController::getCurrentWaypointIndex() const {
  return current_waypoint_index_;
}

void PurePursuitController::setLookaheadDistance(double lookAhead) {
  lookahead_distance_ = lookAhead;
}
