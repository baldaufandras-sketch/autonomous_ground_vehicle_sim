#include "agv_sim/stanley_controller.hpp"

StanleyController::StanleyController(const StanleyConfig &config)
    : config_{config} {}

ControllerDebugInfo StanleyController::getControllerDebugInfo() const {
  return debug_info_;
}

VehicleInput StanleyController::computeControl(const VehicleState &state,
                                               const Path &path) {
  PathSegment current_segment = findClosestSegment(path, state);
  Waypoint closest_point{closestPointToSegment(current_segment, state)};
  double lateral_error =
      std::hypot(state.x - closest_point.x, state.y - closest_point.y);
  double steering_angle_request =
      (current_segment.heading - state.heading) +
      atan(config_.stanley_gain * lateral_error / (state.speed + 1));
  // TODO: with speed = -1 thos can be division by zero!
  // std::cout << "state heading: " << state.heading << std::endl;
  // std::cout << "lateral error: " << lateral_error << std::endl;
  return VehicleInput{0, steering_angle_request};
}

Waypoint StanleyController::closestPointToSegment(const PathSegment &segment,
                                                  const VehicleState &state) {
  // length(std::hypot(end.x - start.x, end.y - start.y)),
  double diff_direction =
      dotBetweenSegments(segment.start, segment.end, segment.start,
                         Waypoint{state.x, state.y}) /
      (segment.length * segment.length);
  if (diff_direction <= 0) {
    return segment.start;
  }
  if (diff_direction >= 1) {
    return segment.end;
  }
  Waypoint projection_point{};
  projection_point.x =
      segment.start.x + diff_direction * (segment.end.x - segment.start.x);
  projection_point.y =
      segment.start.y + diff_direction * (segment.end.y - segment.start.y);
  return projection_point;
}

PathSegment StanleyController::findClosestSegment(const Path &path,
                                                  const VehicleState &state) {
  return path.getSegments()[0];
}
