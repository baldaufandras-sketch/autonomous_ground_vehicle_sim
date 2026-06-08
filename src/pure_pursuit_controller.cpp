#include "agv_sim/pure_pursuit_controller.hpp"
#include <cmath>
#include <iostream>
#include <optional>

PurePursuitController::PurePursuitController(double lookahead_distance,
                                             double wheelbase)
    : lookahead_distance_{lookahead_distance}, wheelbase_{wheelbase} {}

VehicleInput
PurePursuitController::computeControl(const VehicleState &state,
                                      const std::vector<Waypoint> &waypoints) {
  VehicleInput controllerOutput{};
  // First current waypoint index shall be updated
  updateCurrentWaypoint(state, waypoints);

  // After the target waypoint is known, the lookahead point can be calculated
  Waypoint target_point = findLookaheadPoint(state, waypoints);

  // After the lookahead point is known, the steering angle can be calculated
  double steering_angle = computeSteeringAngle(state, target_point);
  controllerOutput.steering_angle = steering_angle;
  if (current_waypoint_index_ == (waypoints.size() - 1)) {
    controllerOutput.acceleration = -3;
  } else {
    controllerOutput.acceleration = 0;
  }
  return controllerOutput;
}

Waypoint PurePursuitController::findLookaheadPoint(
    const VehicleState &state, const std::vector<Waypoint> &waypoints) const {
  Waypoint lookahead_point{};
  if (current_waypoint_index_ == 0) {
    lookahead_point = waypoints[0];
    return lookahead_point;
  }
  std::optional<Waypoint> intersection = findLookaheadPointOnPathSegment(
      state, waypoints[current_waypoint_index_ - 1],
      waypoints[current_waypoint_index_], lookahead_distance_);

  if (intersection.has_value()) {
    return intersection.value();
    // use target
  } else {
    return waypoints[current_waypoint_index_]; // no intersection: controller
                                               // decides fallback behavior
  }
}

void PurePursuitController::updateCurrentWaypoint(
    const VehicleState &state, const std::vector<Waypoint> &waypoints) {
  size_t target_index = PurePursuitController::current_waypoint_index_;
  size_t max_waypoint = waypoints.size() - 1;
  double distance_to_waypoint{};
  double dx;
  double dy;
  for (size_t i = target_index; i < max_waypoint; i++) {
    dx = state.x - waypoints[i].x;
    dy = state.y - waypoints[i].y;
    distance_to_waypoint = std::sqrt(dx * dx + dy * dy);
    if (distance_to_waypoint >= lookahead_distance_) {
      break;
    }
    if (target_index == max_waypoint) {
      current_waypoint_index_ = max_waypoint;
      break;
    }
    current_waypoint_index_ += 1;
  }
}

double PurePursuitController::computeSteeringAngle(
    const VehicleState &state, const Waypoint &target_point) const {
  // TODO
  double dx = target_point.x - state.x;
  double dy = target_point.y - state.y;

  double target_heading = std::atan2(dy, dx);
  double alpha = target_heading - state.heading;

  double steering_angle_target =
      std::atan2(2 * wheelbase_ * std::sin(alpha), lookahead_distance_);
  return steering_angle_target;
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
