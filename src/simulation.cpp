#include "agv_sim/simulation.hpp"
#include "agv_sim/pure_pursuit_controller.hpp"

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
