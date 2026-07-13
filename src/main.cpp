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
  SimulationConfig simConfig{15, 0.1};
  VehicleLimits bmw_limits{constants::pi / 6, constants::pi};
  VehicleState init_state{.x = 0.0,
                          .y = -5.0,
                          .speed = 10,
                          .heading = -constants::pi / 6,
                          .steering_angle = 0.0};

  VehicleSpec bmw_spec{init_state, bmw_limits};
  std::string name{"basic_dstanley"};
  ControllerSpec stanley_2_0 = makeStanleySpec(name, StanleyConfig{2});
  /*PurePursuitConfig controllerConfig{.lookahead_distance = lookahead_distance,
                                     .wheelbase = bmw.getWheelbase(),
                                     .fallback_steering_angle =
                                         constants::pi / 9};

  PurePursuitController pursuit_controller{controllerConfig};*/

  /*std::vector<Waypoint> waypoints{{0.0, 0.0},   {10.0, 0.0}, {20.0, 3.0},
                                  {30.0, -6.0}, {40.0, 6.0}, {50.0, -6.0},
                                  {60.0, 3.0},  {70.0, 0.0}};*/

  // std::vector<Waypoint> waypoints{{0.0, 0.0}, {100.0, 50.0}};
  /* struct Scenario {
    std::string name;
    VehicleSpec vehicle;
    std::filesystem::path path_file;
    SimulationConfig simulation_config;
  };
  */
  std::filesystem::path wide_turn_path{"data/paths/wide_turn_path.csv"};
  Scenario basic_stanley{"basic_stanley", bmw_spec, wide_turn_path, simConfig};

  std::cout << "Starting simulation..." << std::endl;
  SimulationLog log = runScenario(basic_stanley, stanley_2_0);
  writeLogToFile(log);
}