#include "agv_sim/simulation.hpp"
#include "agv_sim/pure_pursuit_controller.hpp"

SimulationLog runSimulation(Vehicle &vehicle,
                            const std::vector<Waypoint> &waypoints,
                            const SimulationConfig &config,
                            PurePursuitController &controller) {
  SimulationLog log;
  VehicleInput nextInput{.acceleration = 0, .steering_angle = 0};

  double time = 0.0;

  while (time <= config.end_time) {
    // add current state to log
    log.push_back({time, vehicle.getX(), vehicle.getY(), vehicle.getSpeed(),
                   vehicle.getHeading(), vehicle.getAcceleration(),
                   vehicle.getSteeringAngle()});

    // update controller
    nextInput = controller.computeControl(vehicle.getVehicleState(), waypoints);

    // update vehicle state
    vehicle.update(config.dt);

    time += config.dt.seconds;
  }

  return log;
}
