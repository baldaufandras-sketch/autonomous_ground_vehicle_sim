#include "agv_sim/path_geometry.hpp"
#include <iostream>

// The lookahead point can be found on the line
// connecting the current target
// waypoint and the previous waypoint offset and incline is of this line
std::optional<Waypoint> findLookaheadPointOnPathSegment(
    const VehicleState &state, const Waypoint &wp_prev,
    const Waypoint &wp_current, const double lookahead_distance) {
  Waypoint lookahead_point{};
  // double x_current_waypoint = wp_current.x;
  // double y_current_waypoint = wp_current.y;
  // double x_prev_waypoint = wp_prev.x;
  // double y_prev_waypoint = wp_prev.y;

  if (std::abs(wp_current.y - wp_prev.y) < 0.05) {
    // If delta_y approaches 0, atan(delta_x/delta_y)later on will not be
    // calculable
    lookahead_point.y = (wp_current.y + wp_prev.y) / 2;
    double delta_y = wp_current.y - state.y;
    double direction{1};
    if (wp_current.x < state.x) {
      direction = -1;
    }
    lookahead_point.x = state.x + direction * std::sqrt(lookahead_distance *
                                                            lookahead_distance -
                                                        delta_y * delta_y);
    return lookahead_point;
  }

  double incline = (wp_current.x - wp_prev.x) / (wp_current.y - wp_prev.y);
  double offset = wp_prev.x - incline * wp_prev.y;
  // The lookahead point is also exactly at the lookahead distance away from
  // the vehicle position Solving the equations results in a quadratical
  // equation, where the constansts are the following:
  double a = incline * incline + 1;
  double b = 2 * (incline * offset - incline * state.x - state.y);
  double c = offset * offset + state.x * state.x + state.y * state.y -
             2 * offset * state.x - lookahead_distance * lookahead_distance;

  double discriminant = b * b - 4 * a * c;
  if (discriminant < 0) {
    // there is no intersection
    return std::nullopt;
  }
  // And solving the quadratical equation for x:
  Waypoint solution_1{};
  Waypoint solution_2{};
  solution_1.y = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
  solution_2.y = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
  solution_1.x = incline * solution_1.y + offset;
  solution_2.x = incline * solution_2.y + offset;

  /*if ((wp_prev.y <= solution_1.y && solution_1.y <= wp_current.y) ||
      (wp_prev.y > solution_1.y && solution_1.y > wp_current.y)) {
    lookahead_point = solution_1;
    std::cout << "y_1 was chosen" << std::endl;
  } else {
    std::cout << "y_2 was chosen" << std::endl;
    lookahead_point = solution_2;
  }*/
  return chooseSolution(wp_prev, wp_current, solution_1, solution_2);
}

Waypoint chooseSolution(const Waypoint wp_prev, const Waypoint wp_current,
                        const Waypoint solution_1, const Waypoint solution_2) {
  double dx = wp_current.x - wp_prev.x;
  double dy = wp_current.y - wp_prev.y;
  double segment_length_sq = dx * dx + dy * dy;
  double t_1 =
      ((solution_1.x - wp_prev.x) * dx + (solution_1.y - wp_prev.y) * dy) /
      segment_length_sq;
  double t_2 =
      ((solution_2.x - wp_prev.x) * dx + (solution_2.y - wp_prev.y) * dy) /
      segment_length_sq;
  bool is_solution_1_on_segment = (0 <= t_1) && (t_1 <= 1);
  bool is_solution_2_on_segment = (0 <= t_2) && (t_2 <= 1);
  if (!is_solution_1_on_segment) {
    return solution_2;
  } else if (is_solution_1_on_segment && !(is_solution_2_on_segment)) {
    return solution_1;
  } else if (t_1 > t_2) {
    return solution_1;
  } else {
    return solution_2;
  }
}

double dotBetweenSegments(const Waypoint &p1, const Waypoint &p2,
                          const Waypoint &q1, const Waypoint &q2) {
  return (p2.x - p1.x) * (q2.x - q1.x) + (p2.y - p1.y) * (q2.y - q1.y);
}