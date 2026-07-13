#include "agv_sim/csv_handling.hpp"
#include "agv_sim/simulation.hpp"

void writeLogToFile(const SimulationLog &log) {
  std::ofstream file("data/simulation_log.csv");

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

Path fileToWaypoints(const std::filesystem::path file_location) {
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