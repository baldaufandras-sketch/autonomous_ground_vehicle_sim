#include "agv_sim/simulation.hpp"
#include "agv_sim/pure_pursuit_controller.hpp"
#include <iostream>

SimulationLog runSimulation(Vehicle &vehicle,
                            const std::vector<Waypoint> &waypoints,
                            const SimulationConfig &config,
                            PurePursuitController &controller) {
  SimulationLog log;

  double time = 0.0;

  while (time <= config.end_time) {
    // add current state to log
    log.push_back({time, vehicle.getX(), vehicle.getY(), vehicle.getSpeed(),
                   vehicle.getHeading(), vehicle.getAcceleration(),
                   vehicle.getSteeringAngle()});

    // update controller
    const VehicleInput nextInput =
        controller.computeControl(vehicle.getVehicleState(), waypoints);
    // update vehicle input
    vehicle.setInput(nextInput);
    // update vehicle state
    vehicle.update(config.dt);

    time += config.dt.seconds;
  }

  return log;
}

namespace {
void validateState(const VehicleState &state) {
  if (std::isnan(state.x) || std::isnan(state.y) || std::isnan(state.yaw) ||
      std::isnan(state.velocity)) {
    throw std::runtime_error("Vehicle state contains NaN");
  }
}
} // namespace