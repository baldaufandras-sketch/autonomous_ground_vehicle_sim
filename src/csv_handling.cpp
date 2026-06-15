#include "agv_sim/csv_handling.hpp"

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