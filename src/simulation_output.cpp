#include "agv_sim/simulation_output.hpp"
#include "agv_sim/simulation.hpp"

void writeLogToFile(SimulationLog log,
                    const std::filesystem::path folder_location) {
  std::filesystem::path file_location = folder_location / "log.csv";
  std::ofstream file(file_location);
  if (!file.is_open()) {
    std::cerr << "Failed to open output file: "
              << std::filesystem::absolute(file_location) << '\n';
    return;
  }
  file << SimulationSample::csvHeader() << "\n";

  for (const auto &sample : log) {
    file << sample.toCsvRow() << "\n";
  }
}

void waypointsToFile(const std::vector<Waypoint> &waypoints) {
  std::ofstream waypoint_file("data/waypoints.csv");

  waypoint_file << "x,y\n";

  for (const auto &wp : waypoints) {
    waypoint_file << wp.x << "," << wp.y << "\n";
  }
}

Path fileToWaypoints(const std::filesystem::path &file_location) {
  std::vector<Waypoint> waypoints{};
  std::ifstream file(file_location);

  if (!file.is_open()) {
    throw std::runtime_error("Could not open file: " + file_location.string());
  }
  std::string line;

  // Skip header
  std::getline(file, line);

  while (std::getline(file, line)) {
    std::stringstream line_stream(line);

    std::string x_string;
    std::string y_string;

    std::getline(line_stream, x_string, ',');
    std::getline(line_stream, y_string, ',');
    // Process one complete CSV line here
    double x = std::stod(x_string);
    double y = std::stod(y_string);
    Waypoint new_waypoint{x, y};
    waypoints.push_back(new_waypoint);
  }

  return Path{waypoints};
}

void writeScenarioToYaml(ScenarioRunManifest manifest,
                         const std::filesystem::path folder_location) {

  std::filesystem::path file_location = folder_location / "scenario_info.yaml";
  std::ofstream file(file_location);
  file << "Scenario name: " << manifest.scenario_name << "\n";
  file << "Waypoint soure location: " << manifest.path_original_location
       << "\n";
}

void writeResults(const std::vector<ScenarioRunResult> &scenario_list) {
  std::filesystem::path results_directory = "data/results";
  std::filesystem::path run_directory =
      results_directory / ("run_" + createTimestamp());
  //
  for (auto scenario : scenario_list) {
    std::filesystem::path scenario_directory =
        run_directory / "scenarios" / scenario.scenario_info.scenario_name;
    std::filesystem::create_directories(scenario_directory);
    writeLogToFile(scenario.log, scenario_directory);
    writeScenarioToYaml(scenario.scenario_info, scenario_directory);
    std::filesystem::path path_target_loc = scenario_directory / "path.csv";
    std::filesystem::copy(scenario.scenario_info.path_original_location,
                          path_target_loc);
  }
}

std::string createTimestamp() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t current_time = std::chrono::system_clock::to_time_t(now);

  std::tm local_time{};

#ifdef _WIN32
  localtime_s(&local_time, &current_time);
#else
  localtime_r(&current_time, &local_time);
#endif

  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y-%m-%d_%H-%M-%S");

  return stream.str();
}