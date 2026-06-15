#include "agv_sim/pure_pursuit_controller.hpp"
#include "agv_sim/constants.hpp"
#include <cmath>
#include <iostream>
#include <optional>

PurePursuitController::PurePursuitController(const PurePursuitConfig &config)
    : config_{config} {}

VehicleInput
PurePursuitController::computeControl(const VehicleState &state,
                                      const std::vector<Waypoint> &waypoints) {
  VehicleInput controllerOutput{};
  // First current waypoint index shall be updated
  updateCurrentWaypoint(state, waypoints);
  // After the target waypoint is known, the lookahead point can be calculated
  Waypoint target_point = findLookaheadPoint(state, waypoints);
  // After the lookahead point is known, the Control Output can be calculated
  controllerOutput.steering_angle_request =
      computeSteeringAngle(state, target_point);
  controllerOutput.acceleration = computeAcceleration(waypoints);
  debug_info_ = {
      .current_waypoint_index = current_waypoint_index_,
      .target_x = target_point.x,
      .target_y = target_point.y,
      .pursuit_controller_alpha = 0.0,
      .steering_angle_request = controllerOutput.steering_angle_request,
  };

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
      waypoints[current_waypoint_index_], config_.lookahead_distance);

  if (intersection.has_value()) {
    return intersection.value();
    // use target
  } else {
    // no intersection can be found, next waypoint is the fallback
    return waypoints[current_waypoint_index_];
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
    if (distance_to_waypoint >= config_.lookahead_distance) {
      break;
    }
    if (target_index == max_waypoint) {
      current_waypoint_index_ = max_waypoint;
      break;
    }
    current_waypoint_index_ += 1;
  }
}

double
PurePursuitController::computeSteeringAngle(const VehicleState &state,
                                            const Waypoint &target_point) {
  double dx = target_point.x - state.x;
  double dy = target_point.y - state.y;
  double alpha{0};
  double target_heading = std::atan2(dy, dx);
  alpha = target_heading - state.heading;
  if (std::abs(alpha) > (constants::pi / 2) &&
      std::abs(alpha) < (1.5 * constants::pi)) {
    // If the vehicle faces away from the waypoint, a constant steering angle is
    // applied to turn back
    return std::copysign(config_.fallback_steering_angle, alpha);
  }

  double steering_angle_target = std::atan2(
      2 * config_.wheelbase * std::sin(alpha), config_.lookahead_distance);

  return steering_angle_target;
}

double PurePursuitController::computeAcceleration(
    const std::vector<Waypoint> &waypoints) const {
  if (current_waypoint_index_ == (waypoints.size() - 1)) {
    return -3;
  } else {
    return 0;
  }
}

void PurePursuitController::reset(std::size_t start_index = 0) {
  current_waypoint_index_ = start_index;
}

std::size_t PurePursuitController::getCurrentWaypointIndex() const {
  return current_waypoint_index_;
}

void PurePursuitController::setLookaheadDistance(double lookAhead) {
  config_.lookahead_distance = lookAhead;
}

ControllerDebugInfo PurePursuitController::getControllerDebugInfo() const {
  return debug_info_;
}
