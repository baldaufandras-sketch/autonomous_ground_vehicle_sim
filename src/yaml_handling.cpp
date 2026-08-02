#include "agv_sim/yaml_handling.hpp"

Scenario loadScenarioFromYaml(const std::filesystem::path &yaml_path) {
  try {
    const YAML::Node root = YAML::LoadFile(yaml_path.string());
  } catch (const YAML::BadFile &error) {
    throw std::runtime_error("Could not open scenario file: " +
                             yaml_path.string() + "\n" + error.what());
  }
  try {
    Scenario scenario;

    // Specyfing controller
    ControllerSpec controller_spec{};
    std::string controller_type = root["controller"]["type"].as<std::string>();
    std::string controller_name = root["controller"]["name"].as<std::string>();
    if (controller_type == "Stanley") {
      double stanley_gain = root["controller"]["gain"].as<double>();
      controller_spec =
          makeStanleySpec(controller_name, StanleyConfig{stanley_gain});
    } else if (controller_type == "PurePursuit") {
      double lookahead = root["controller"]["lookahead distance"].as<double>();
      double fallback_steering =
          root["controller"]["fallback steering angle"].as<double>();
      controller_spec = makePurePursuitSpec(
          controller_name, PurePursuitConfig{lookahead, 1, fallback_steering});
    } else {
      throw std::runtime_error("Invalid controller type: " + controller_type);
    }
    scenario.controller_spec = controller_spec;

    // Specyfing simulation config
    double simulation_length = root["simulation"]["length"].as<double>();
    double timestep = root["simulation"]["timestep"].as<double>();
    SimulationConfig simulation_config{
        .end_time = simulation_length,
        .dt = TimeStep{timestep},
    };
    scenario.simulation_config = simulation_config;

    //  Specifying vehicle

    double x = root["vehicle"]["initial_position"][0].as<double>();
    double y = root["vehicle"]["initial_position"][1].as<double>();
    double init_speed = root["vehicle"]["initial_speed"].as<double>();
    double init_heading = root["vehicle"]["initial_heading"].as<double>();

    double max_steering =
        root["vehicle limits"]["maximum steering angle in radian"].as<double>();
    double max_steering_grad =
        root["vehicle limits"]["maximum steering angle gradient in radian"]
            .as<double>();

    VehicleState initial_state{.x = x,
                               .y = y,
                               .speed = init_speed,
                               .heading = init_heading,
                               .steering_angle = init_heading}; // TODO
    VehicleLimits veh_limits{.maxSteeringAngle = max_steering,
                             .maxSteeringAngleGradient = max_steering_grad};

    VehicleSpec vehicle_spec{.initial_state = initial_state,
                             .vehicle_limits = veh_limits};
    scenario.vehicle = vehicle_spec;

    // Specifying name and path location
    scenario.name = root["name"].as<std::string>();
    const std::string path_path = root["path_file_location"].as<std::string>();
    scenario.path_file = std::filesystem::path{path_path};

    return scenario;
  } catch (const YAML::Exception &error) {
    throw std::runtime_error("Invalid scenario YAML in " + yaml_path.string() +
                             "\n" + error.what());
  }
}