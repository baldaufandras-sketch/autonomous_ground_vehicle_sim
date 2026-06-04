#include "agv_sim/pure_pursuit_controller.hpp"
#include <cmath>
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
  Waypoint lookahead_point{};
  if (current_waypoint_index_ == 0) {
    lookahead_point = waypoints[0];
    return lookahead_point;
  }
  // The lookahead point can be found on the line connecting the current target
  // waypoint and the previous waypoint offset and incline is of this line
  double x_current_waypoint = waypoints[current_waypoint_index_].x;
  double y_current_waypoint = waypoints[current_waypoint_index_].y;
  double x_prev_waypoint = waypoints[current_waypoint_index_ - 1].x;
  double y_prev_waypoint = waypoints[current_waypoint_index_ - 1].y;

  if (std::abs(y_current_waypoint - y_prev_waypoint) < 0.05) {
    lookahead_point.y = (y_current_waypoint + y_prev_waypoint) / 2;
    double delta_y = y_current_waypoint - state.y;
    double direction{1};
    if (x_current_waypoint < state.x) {
      direction = -1;
    }
    lookahead_point.x =
        state.x +
        direction * std::sqrt(lookahead_distance_ * lookahead_distance_ -
                              delta_y * delta_y);
    return lookahead_point;
  }

  double incline = (x_current_waypoint - x_prev_waypoint) /
                   (y_current_waypoint - y_prev_waypoint);
  double offset = x_prev_waypoint - incline * y_prev_waypoint;
  // The lookahead point is also exactly at the lookahead distance away from the
  // vehicle position Solving the equations results in a quadrazical equation,
  // where the constanst are the following:
  double a = incline * incline + 1;
  double b = 2 * (incline * offset - incline * state.x - state.y);
  double c = offset * offset + state.x * state.x + state.y * state.y -
             2 * offset * state.x - lookahead_distance_ * lookahead_distance_;
  // And solving the quadratical equation for x:
  double y_1 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
  double y_2 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
  // std::cout << "first solution: " << x_1 << std::endl;
  // std::cout << "second solution: " << x_2 << std::endl;
  //  The real lookahead point is always between the vehicle and the next
  //  targetpoint
  if ((state.y <= y_1 && y_1 <= y_current_waypoint) ||
      (y_current_waypoint <= y_1 && y_1 <= state.y)) {
    lookahead_point.y = y_1;
  } else {
    lookahead_point.y = y_2;
  }
  lookahead_point.x = incline * lookahead_point.y + offset;
  return lookahead_point;
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
