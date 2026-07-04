#include "agv_sim/simulation.hpp"
#include <cmath>
#include <iostream>

namespace {
bool validateState(const VehicleState &state) {
  return std::isfinite(state.x) && std::isfinite(state.y) &&
         std::isfinite(state.heading) && std::isfinite(state.speed);
}

bool validateInput(const VehicleInput &input) {
  return std::isfinite(input.acceleration) &&
         std::isfinite(input.steering_angle_request);
}
} // namespace

SimulationLog runSimulation(Vehicle &vehicle, const Path &path,
                            const SimulationConfig &config,
                            StanleyController &controller) {
  SimulationLog log;

  double time = 0.0;

  while (time <= config.end_time) {
    // add current state to log

    // update controller
    const VehicleInput nextInput =
        controller.computeControl(vehicle.getVehicleState(), path);
    // update vehicle input
    vehicle.setInput(nextInput);
    log.push_back(makeSimulationSample(time, vehicle,
                                       controller.getControllerDebugInfo()));
    // update vehicle state
    vehicle.update(config.dt);

    if (!validateState(vehicle.getState()) || !(validateInput(nextInput))) {
      std::cerr << "Invalid state at t=" << time << "\n";
      log.push_back(makeSimulationSample(time, vehicle,
                                         controller.getControllerDebugInfo()));
      break;
    }
    time += config.dt.seconds;
  }

  return log;
}

SimulationSample
makeSimulationSample(double time, const Vehicle &vehicle,
                     const ControllerDebugInfo &controller_info) {
  return SimulationSample{
      .time = time,
      .x = vehicle.getX(),
      .y = vehicle.getY(),
      .speed = vehicle.getSpeed(),
      .heading = vehicle.getHeading(),
      .acceleration = vehicle.getAcceleration(),
      .steering_angle_actual = vehicle.getSteeringAngle(),
      .steering_angle_request = controller_info.steering_angle_request,
      .target_x = controller_info.target_x,
      .target_y = controller_info.target_y,
      .pursuit_controller_alpha = controller_info.pursuit_controller_alpha,
      .current_waypoint_index = controller_info.current_waypoint_index};
}
