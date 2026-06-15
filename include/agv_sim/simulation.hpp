#pragma once

#include "agv_sim/pure_pursuit_controller.hpp"
#include "agv_sim/simulation_config.hpp"
#include "agv_sim/vehicle.hpp"
#include "agv_sim/waypoint.hpp"
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
  size_t current_waypoint_index;

  static std::string csvHeader() {
    return "time,x,y,speed,heading,acceleration,steering_angle_actual,steering_"
           "angle_request, target_x, target_y,pursuit_controller_alpha,"
           "current_waypoint_index";
  }

  std::string toCsvRow() const {
    std::ostringstream row;
    row << time << "," << x << "," << y << "," << speed << "," << heading << ","
        << acceleration << "," << steering_angle_actual << ","
        << steering_angle_request << "," << target_x << "," << target_y << ","
        << pursuit_controller_alpha << "," << current_waypoint_index;

    return row.str();
  }
};

using SimulationLog = std::vector<SimulationSample>;

SimulationLog runSimulation(Vehicle &vehicle,
                            const std::vector<Waypoint> &waypoints,
                            const SimulationConfig &config,
                            PurePursuitController &controller);

SimulationSample
makeSimulationSample(double time, const Vehicle &vehicle,
                     const ControllerDebugInfo &controller_info);