#include "agv_sim/constants.hpp"
#include "agv_sim/csv_handling.hpp"
#include "agv_sim/pure_pursuit_controller.hpp"
#include "agv_sim/simulation.hpp"
#include "agv_sim/vehicle.hpp"

#include <vector>

int main() {
  double lookahead_distance{1};
  SimulationConfig simConfig{15, 0.01};
  VehicleState init_state{
      .x = -1.0, .y = 0.0, .speed = 10, .heading = constants::pi};
  Vehicle bmw{init_state};
  PurePursuitConfig controllerConfig{.lookahead_distance = 1,
                                     .wheelbase = bmw.getWheelbase(),
                                     .fallback_steering_angle =
                                         constants::pi / 9};
  PurePursuitController pursuit_controller{controllerConfig};
  std::vector<Waypoint> waypoints{{0.0, 0.0},   {10.0, 0.0}, {20.0, 3.0},
                                  {30.0, -3.0}, {40.0, 3.0}, {50.0, -3.0},
                                  {60.0, 3.0},  {70.0, 0.0}};
  bmw.setSteeringAngle(0);
  SimulationLog log =
      runSimulation(bmw, waypoints, simConfig, pursuit_controller);
  writeLogToFile(log);
  waypointsToFile(waypoints);
}