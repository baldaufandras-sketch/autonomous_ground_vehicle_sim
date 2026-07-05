#include "agv_sim/constants.hpp"
#include "agv_sim/csv_handling.hpp"
#include "agv_sim/pure_pursuit_controller.hpp"
#include "agv_sim/simulation.hpp"
#include "agv_sim/stanley_controller.hpp"
#include "agv_sim/vehicle.hpp"

#include <iostream>
#include <vector>

int main() {
  double lookahead_distance{3};
  SimulationConfig simConfig{10, 0.1};
  VehicleLimits bmw_limits{constants::pi / 6, constants::pi};
  VehicleState init_state{.x = 0.0,
                          .y = -5.0,
                          .speed = 10,
                          .heading = -constants::pi / 6,
                          .steering_angle = 0.0};

  Vehicle bmw{init_state, bmw_limits};

  /*PurePursuitConfig controllerConfig{.lookahead_distance = lookahead_distance,
                                     .wheelbase = bmw.getWheelbase(),
                                     .fallback_steering_angle =
                                         constants::pi / 9};

  PurePursuitController pursuit_controller{controllerConfig};*/

  StanleyConfig stanley_config{2};

  std::vector<Waypoint> waypoints{{0.0, 0.0},   {10.0, 0.0}, {20.0, 3.0},
                                  {30.0, -6.0}, {40.0, 6.0}, {50.0, -6.0},
                                  {60.0, 3.0},  {70.0, 0.0}};

  // std::vector<Waypoint> waypoints{{0.0, 0.0}, {100.0, 50.0}};
  Path path_to_follow(waypoints);
  StanleyController stanley_controller(stanley_config);

  std::cout << "Starting simulation..." << std::endl;
  SimulationLog log =
      runSimulation(bmw, path_to_follow, simConfig, stanley_controller);
  writeLogToFile(log);
  waypointsToFile(waypoints);
}