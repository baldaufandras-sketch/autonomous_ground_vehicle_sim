#pragma once

#include "agv_sim/path.hpp"
#include "agv_sim/pure_pursuit_controller.hpp"
#include "agv_sim/simulation_config.hpp"
#include "agv_sim/stanley_controller.hpp"
#include "agv_sim/vehicle.hpp"
#include <sstream>

#include <vector>

struct SimulationSample {
  double time;
  double x;
  double y;
  double speed;
  double heading;
  double acceleration;
  double steering_angle_actual;
  double steering_angle_request;
  double target_x;
  double target_y;
  double pursuit_controller_alpha;
  double lateral_error;
  size_t current_waypoint_index;

  static std::string csvHeader() {
    return "time,x,y,speed,heading_deg,acceleration,steering_angle_actual_deg,"
           "steering_angle_request_deg, target_x, "
           "target_y,pursuit_controller_alpha_deg, lateral _error,"
           "current_index";
  }

  std::string toCsvRow() const {
    std::ostringstream row;
    row << time << "," << x << "," << y << "," << speed << ","
        << heading * constants::rad_to_deg << "," << acceleration << ","
        << steering_angle_actual * constants::rad_to_deg << ","
        << steering_angle_request * constants::rad_to_deg << "," << target_x
        << "," << target_y << ","
        << pursuit_controller_alpha * constants::rad_to_deg << ","
        << lateral_error << "," << current_waypoint_index;

    return row.str();
  }
};

using SimulationLog = std::vector<SimulationSample>;

SimulationLog runSimulation(Vehicle &vehicle, const Path &path,
                            const SimulationConfig &config,
                            StanleyController &controller);

SimulationSample
makeSimulationSample(double time, const Vehicle &vehicle,
                     const ControllerDebugInfo &controller_info);