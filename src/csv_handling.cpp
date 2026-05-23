#include "agv_sim/csv_handling.hpp"

void writeLogToFile(const SimulationLog &log) {
  std::ofstream file("data/simulation_log.csv");

  file << "time,x,y,speed,heading,acceleration,steering_angle\n";

  for (const auto &sample : log) {
    file << sample.time << "," << sample.x << "," << sample.y << ","
         << sample.speed << "," << sample.heading << "," << sample.acceleration
         << "," << sample.steering_angle << "\n";
  }
}