#include "agv_sim/simulation.hpp"

SimulationLog runSimulation(Vehicle &vehicle,
                            const std::vector<Waypoint> &waypoints,
                            const SimulationConfig &config) {
  SimulationLog log;

  double time = 0.0;

  while (time <= config.end_time) {
    log.push_back({time, vehicle.getX(), vehicle.getY(), vehicle.getSpeed(),
                   vehicle.getHeading(), vehicle.getAcceleration(),
                   vehicle.getSteeringAngle()});
    vehicle.update(config.dt);

    time += config.dt.seconds;
  }

  return log;
}
