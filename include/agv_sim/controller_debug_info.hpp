#pragma once
#include <cstddef>

struct ControllerDebugInfo {
  std::size_t current_waypoint_index{0};
  double target_x{0};
  double target_y{0};
  double pursuit_controller_alpha{0};
  double steering_angle_request{0};
  double lateral_error{0};
};