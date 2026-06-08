#include "agv_sim/path_geometry.hpp"
#include <iostream>

// The lookahead point can be found on the line
// connecting the current target
// waypoint and the previous waypoint offset and incline is of this line
std::optional<Waypoint> findLookaheadPointOnPathSegment(
    const VehicleState &state, const Waypoint &waypoint_1,
    const Waypoint &waypoint_2, const double lookahead_distance) {
  Waypoint lookahead_point{};
  double x_current_waypoint = waypoint_2.x;
  double y_current_waypoint = waypoint_2.y;
  double x_prev_waypoint = waypoint_1.x;
  double y_prev_waypoint = waypoint_1.y;

  if (std::abs(y_current_waypoint - y_prev_waypoint) < 0.05) {
    lookahead_point.y = (y_current_waypoint + y_prev_waypoint) / 2;
    double delta_y = y_current_waypoint - state.y;
    double direction{1};
    if (x_current_waypoint < state.x) {
      direction = -1;
    }
    lookahead_point.x = state.x + direction * std::sqrt(lookahead_distance *
                                                            lookahead_distance -
                                                        delta_y * delta_y);
    return lookahead_point;
  }

  double incline = (x_current_waypoint - x_prev_waypoint) /
                   (y_current_waypoint - y_prev_waypoint);
  double offset = x_prev_waypoint - incline * y_prev_waypoint;
  // The lookahead point is also exactly at the lookahead distance away from
  // the vehicle position Solving the equations results in a quadrazical
  // equation, where the constanst are the following:
  double a = incline * incline + 1;
  double b = 2 * (incline * offset - incline * state.x - state.y);
  double c = offset * offset + state.x * state.x + state.y * state.y -
             2 * offset * state.x - lookahead_distance * lookahead_distance;

  double discriminant = b * b - 4 * a * c;
  if (discriminant < 0) {
    return std::nullopt;
  }
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