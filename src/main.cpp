#include "agv_sim/simulation.hpp"
#include "agv_sim/vehicle.hpp"
#include <fstream>
#include <iostream>
#include <vector>

int main() {
  constexpr double pi = 3.14159265358979323846;
  SimulationConfig config{10, 0.1};
  VehicleState init_state{.x = 0.0, .y = 5.0, .speed = 4.0, .heading = pi};
  Vehicle bmw{init_state};
  std::vector<Waypoint> waypoints{
      {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}, {4.0, 0.0}};
  bmw.setSteeringAngle(pi / 180);
  SimulationLog log = runSimulation(bmw, waypoints, config);

  std::ofstream file("data/simulation_log.csv");

  file << "time,x,y,speed,heading,acceleration,steering_angle\n";

  for (const auto &sample : log) {
    file << sample.time << "," << sample.x << "," << sample.y << ","
         << sample.speed << "," << sample.heading << "," << sample.acceleration
         << "," << sample.steering_angle << "\n";
  }
}