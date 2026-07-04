#pragma once
#include <cstddef>

struct ControllerDebugInfo {
  std::size_t current_waypoint_index{};
  double target_x{};
  double target_y{};
  double pursuit_controller_alpha{};
  double steering_angle_request{};
};