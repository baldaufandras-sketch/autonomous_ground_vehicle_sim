#include "agv_sim/simulation.hpp"
#include "agv_sim/csv_handling.hpp"
#include <cmath>
#include <iostream>

bool validateState(const VehicleState &state) {
  return std::isfinite(state.x) && std::isfinite(state.y) &&
         std::isfinite(state.heading) && std::isfinite(state.speed);
}

bool validateInput(const VehicleInput &input) {
  return std::isfinite(input.acceleration) &&
         std::isfinite(input.steering_angle_request);
}

SimulationLog runSimulation(Vehicle &vehicle, const Path &path,
                            const SimulationConfig &config,
                            IPathTrackingController &controller) {
  SimulationLog log;
  std::cout << "run simulation did start" << std::endl;
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
      .lateral_error = controller_info.lateral_error,
      .current_waypoint_index = controller_info.current_waypoint_index};
}

SimulationLog runScenario(Scenario &scenario, ControllerSpec &controller_spec) {
  Vehicle vehicle{scenario.vehicle.initial_state,
                  scenario.vehicle.vehicle_limits};
  Path path{fileToWaypoints(scenario.path_file)};
  auto controller = createController(controller_spec);
  SimulationLog log =
      runSimulation(vehicle, path, scenario.simulation_config, *controller);
  return SimulationLog{};
}
