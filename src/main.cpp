#include "agv_sim/constants.hpp"
#include "agv_sim/pure_pursuit_controller.hpp"
#include "agv_sim/simulation.hpp"
#include "agv_sim/simulation_output.hpp"
#include "agv_sim/stanley_controller.hpp"
#include "agv_sim/vehicle.hpp"
#include <cstdlib>
#include <iostream>
#include <vector>

int main(int argc, char *argv[]) {
  std::string command;
  if (argc < 2) {
    command = "all";
  } else {
    command = argv[1];
  }
  std::filesystem::path folder = "data/scenarios";
  std::vector<std::filesystem::path> yaml_list{};
  std::vector<Scenario> scenario_list{};
  for (const auto &entry : std::filesystem::directory_iterator(folder)) {
    yaml_list.push_back(entry.path());
  }

  if (command == "list") {
    for (auto scenario : yaml_list) {
      std::cout << scenario << '\n';
    }
    return 1;

  } else if (command == "all") {

    for (const auto &entry : yaml_list) {
      std::cout << "loading scenario " << entry << std::endl;
      scenario_list.push_back(loadScenarioFromYaml(entry));
    }

  } else {

    bool scenario_found{false};
    for (int i = 1; i < argc; ++i) {
      std::string current_command = argv[i];
      for (const auto &scenario_path : yaml_list) {
        if (scenario_path.stem().string() == current_command) {
          std::cout << "Scenario found. Loading "
                    << scenario_path.stem().string() << std::endl;
          scenario_list.push_back(loadScenarioFromYaml(scenario_path));
          scenario_found = true;
        }
      }
    }
    if (!scenario_found) {
      std::cout << "No scenario found: " << command << std::endl;
    }
  }
  // std::cout << argv[argc - 1] << std::endl;
  std::vector<ScenarioRunResult> results = runScenario(scenario_list);
  std::filesystem::path run_directory = writeResults(results);
  if (std::string(argv[argc - 1]) == "--postprocess") {
    std::cout << "was postprocess" << std::endl;
    std::string command =
        "/home/andras/.venvs/vscode/bin/python postprocessing/postprocess.py " +
        run_directory.filename().string();
    std::cout << command << std::endl;
    int return_code = std::system(command.c_str());
  }
}