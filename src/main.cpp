#include "agv_sim/csv_handling.hpp"
#include "agv_sim/pure_pursuit_controller.hpp"
#include "agv_sim/simulation.hpp"
#include "agv_sim/vehicle.hpp"

#include <vector>

int main() {
  constexpr double pi = 3.14159265358979323846;
  double lookahead_distance{0.2};
  SimulationConfig config{5, 0.1};
  VehicleState init_state{.x = 0.0, .y = 5.0, .speed = 4.0, .heading = 0};
  Vehicle bmw{init_state};
  PurePursuitController pursuit_controller{lookahead_distance,
                                           bmw.getWheelbase()};
  std::vector<Waypoint> waypoints{
      {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}, {4.0, 0.0}};
  bmw.setSteeringAngle(0);
  SimulationLog log = runSimulation(bmw, waypoints, config, pursuit_controller);
  writeLogToFile(log);
}