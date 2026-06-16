#include "agv_sim/constants.hpp"
#include "agv_sim/csv_handling.hpp"
#include "agv_sim/pure_pursuit_controller.hpp"
#include "agv_sim/simulation.hpp"
#include "agv_sim/vehicle.hpp"

#include <iostream>
#include <vector>

int main() {
  double lookahead_distance{3};
  SimulationConfig simConfig{10, 0.1};
  VehicleLimits limits{constants::pi / 60.0, constants::pi / 180};
  VehicleState init_state{
      .x = -1.0, .y = 0.0, .speed = 16, .heading = 0, .steering_angle = 0.0};
  Vehicle bmw{init_state};
  PurePursuitConfig controllerConfig{.lookahead_distance = lookahead_distance,
                                     .wheelbase = bmw.getWheelbase(),
                                     .fallback_steering_angle =
                                         constants::pi / 9};
  PurePursuitController pursuit_controller{controllerConfig};
  std::vector<Waypoint> waypoints{{0.0, 0.0},   {10.0, 0.0}, {20.0, 3.0},
                                  {30.0, -6.0}, {40.0, 6.0}, {50.0, -6.0},
                                  {60.0, 3.0},  {70.0, 0.0}};

  std::cout << "Starting simulation..." << std::endl;
  SimulationLog log =
      runSimulation(bmw, waypoints, simConfig, pursuit_controller);
  writeLogToFile(log);
  waypointsToFile(waypoints);
}