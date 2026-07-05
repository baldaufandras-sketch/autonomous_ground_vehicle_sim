#include "agv_sim/stanley_controller.hpp"

StanleyController::StanleyController(const StanleyConfig &config)
    : config_{config} {}

ControllerDebugInfo StanleyController::getControllerDebugInfo() const {
  return debug_info_;
}

VehicleInput StanleyController::computeControl(const VehicleState &state,
                                               const Path &path) {
  updateCurrentSegmentIndex(path, state);
  PathSegment current_segment = path.getSegments()[current_segment_index_];
  SegmentProjection current_segment_projection =
      projectPointToSegment(current_segment, Waypoint{state.x, state.y});
  Waypoint closest_point{current_segment_projection.closest_point};
  double acceleration_request = 0;
  double lateral_error =
      calculateSignedLateralError(state, closest_point, current_segment);
  double steering_angle_request =
      (current_segment.heading - state.heading) +
      atan(config_.stanley_gain * lateral_error / (state.speed + 1));
  // TODO: with speed = -1 thos can be division by zero!
  if (current_segment_index_ >= (path.getSegments().size() - 1) &&
      current_segment_projection.segment_parameter >= 0.9) {
    acceleration_request = -30;
  }
  debug_info_ = {.current_waypoint_index = current_segment_index_,
                 .steering_angle_request = steering_angle_request,
                 .lateral_error = lateral_error};
  return VehicleInput{acceleration_request, steering_angle_request};
}

void StanleyController::updateCurrentSegmentIndex(const Path &path,
                                                  const VehicleState &state) {
  size_t SEARCH_WINDOW = 5;
  size_t start_index{};
  if (current_segment_index_ > 0) {
    start_index = current_segment_index_ - 1;
  } else {
    start_index = 0;
  }
  size_t end_index = std::min(current_segment_index_ + SEARCH_WINDOW,
                              path.getSegments().size() - 1);

  double best_distance_sq = std::numeric_limits<double>::max();
  double best_index_segment_parameter{0};

  size_t best_index = current_segment_index_;

  for (size_t i = start_index; i <= end_index; ++i) {
    SegmentProjection closest = projectPointToSegment(
        path.getSegments()[i], Waypoint{state.x, state.y});

    if (closest.distance_sq < best_distance_sq) {
      best_distance_sq = closest.distance_sq;
      best_index_segment_parameter = closest.segment_parameter;
      best_index = i;
    }
  }
  if (best_index_segment_parameter >= 0.8) {
    current_segment_index_ = std::clamp(best_index + 1, start_index, end_index);
  } else {
    current_segment_index_ = best_index;
  }
}

double
StanleyController::calculateSignedLateralError(const VehicleState &state,
                                               const Waypoint &closest_point,
                                               const PathSegment &segment) {
  const double segment_dx = segment.end.x - segment.start.x;
  const double segment_dy = segment.end.y - segment.start.y;

  const double vehicle_dx = state.x - closest_point.x;
  const double vehicle_dy = state.y - closest_point.y;

  const double cross = segment_dx * vehicle_dy - segment_dy * vehicle_dx;
  const double distance = std::hypot(vehicle_dx, vehicle_dy);

  if (cross > 0.0) {
    return -distance;
  }

  if (cross < 0.0) {
    return distance;
  }

  return 0.0;
}
