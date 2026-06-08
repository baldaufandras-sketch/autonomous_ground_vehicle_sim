#include "agv_sim/csv_handling.hpp"
#include "agv_sim/pure_pursuit_controller.hpp"
#include "agv_sim/simulation.hpp"
#include "agv_sim/vehicle.hpp"

#include <vector>

int main() {
  constexpr double pi = 3.14159265358979323846;
  double lookahead_distance{1};
  SimulationConfig config{15, 0.1};
  VehicleState init_state{.x = -1.0, .y = 0.0, .speed = 10, .heading = 0};
  Vehicle bmw{init_state};
  PurePursuitController pursuit_controller{lookahead_distance,
                                           bmw.getWheelbase()};
  std::vector<Waypoint> waypoints{{0.0, 0.0},   {10.0, 0.0}, {20.0, 3.0},
                                  {30.0, -3.0}, {40.0, 3.0}, {50.0, -3.0},
                                  {60.0, 3.0},  {70.0, 0.0}};
  bmw.setSteeringAngle(0);
  SimulationLog log = runSimulation(bmw, waypoints, config, pursuit_controller);
  writeLogToFile(log);
  waypointsToFile(waypoints);
}