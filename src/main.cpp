#include "agv_sim/constants.hpp"
#include "agv_sim/pure_pursuit_controller.hpp"
#include "agv_sim/simulation.hpp"
#include "agv_sim/simulation_output.hpp"
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
  std::string name{"basic_stanley"};
  ControllerSpec stanley_2_0 = makeStanleySpec(name, StanleyConfig{2});
  ControllerSpec stanley_0_5 = makeStanleySpec(name, StanleyConfig{0.5});

  std::filesystem::path wide_turn_path{"data/paths/wide_turn_path.csv"};
  std::filesystem::path s_curve{"data/paths/s_curve.csv"};
  Scenario stanley_wide_turn{"stanley_wide_turn", bmw_spec, wide_turn_path,
                             simConfig, stanley_2_0};
  Scenario stanley_s_curve{"stanley_s_curve", bmw_spec, s_curve, simConfig,
                           stanley_2_0};
  std::vector<Scenario> scenario_list{stanley_wide_turn, stanley_s_curve};
  std::cout << "Starting simulation..." << std::endl;
  std::vector<ScenarioRunResult> results = runScenario(scenario_list);
  // SimulationLog log = runScenario(basic_stanley, stanley_2_0);
  double a = 0.0;
  // writeLogToFile(results);
  writeResults(results);
  // writeScenarioToYaml(results);
}