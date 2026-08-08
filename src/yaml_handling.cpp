#include "agv_sim/yaml_handling.hpp"

Scenario loadScenarioFromYaml(const std::filesystem::path &yaml_path) {
  try {
    const YAML::Node root = YAML::LoadFile(yaml_path.string());
  } catch (const YAML::BadFile &error) {
    throw std::runtime_error("Could not open scenario file: " +
                             yaml_path.string() + "\n" + error.what());
  }
  try {
    const YAML::Node root = YAML::LoadFile(yaml_path.string());
    Scenario scenario;
    // Specyfing controller
    ControllerSpec controller_spec{};
    std::string controller_type = readOptional<std::string>(
        root["controller"], "type", "Stanley", "Controller type");
    std::string controller_name = readOptional<std::string>(
        root["controller"], "name", "Default Stanley controller",
        "Controller name");
    if (controller_type == "Stanley") {
      double stanley_gain =
          readOptional(root["controller"], "gain", 1.0, "Stanley gain");
      controller_spec =
          makeStanleySpec(controller_name, StanleyConfig{stanley_gain});
    } else if (controller_type == "PurePursuit") {
      double wheelbase =
          readOptional(root["vehicle"], "wheelbase", 1.0, "Wheelbase");
      double lookahead = readOptional(root["controller"], "lookahead distance",
                                      1.0, "Lookahead distance");
      double fallback_steering =
          readOptional(root["controller"], "fallback steering angle", 0.3,
                       "Fallback steering angle");
      controller_spec = makePurePursuitSpec(
          controller_name, PurePursuitConfig{lookahead, 1, fallback_steering});
    } else {
      throw std::runtime_error("Invalid controller type: " + controller_type);
    }
    scenario.controller_spec = controller_spec;

    // Specyfing simulation config
    double simulation_length =
        readOptional(root["simulation"], "length", 1.0, "Simulation length");
    double timestep = readOptional(root["simulation"], "timestep", 0.1,
                                   "Simluation timestep");
    SimulationConfig simulation_config{
        .end_time = simulation_length,
        .dt = TimeStep{timestep},
    };
    scenario.simulation_config = simulation_config;

    //  Specifying vehicle
    std::vector<double> initial_position = readOptional<std::vector<double>>(
        root["vehicle"], "initial_position", std::vector<double>{0.0, 0.0},
        "Initial position");
    double x = initial_position[0];
    double y = initial_position[1];
    double init_speed =
        readOptional(root["vehicle"], "initial_speed", 4.0, "Initial speed");
    double init_heading = readOptional(root["vehicle"], "initial_heading", 0.0,
                                       "Initial heading");
    double max_steering =
        readOptional(root["vehicle limits"], "maximum steering angle in radian",
                     0.6, "Maximum steering angle");
    double max_steering_grad = readOptional(
        root["vehicle limits"], "maximum steering angle gradient in radian",
        0.6, "Maximum steering angle gradient");

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
    scenario.name =
        readOptional(root, "name", std::string{"kutyafule"}, "Controller name");
    // scenario.name = root["name"].as<std::string>();
    const std::string path_path = root["path_file_location"].as<std::string>();
    scenario.path_file = std::filesystem::path{path_path};

    return scenario;
  } catch (const YAML::Exception &error) {
    throw std::runtime_error("Invalid scenario YAML in " + yaml_path.string() +
                             "\n" + error.what());
  }
}

template <typename T>
T readOptional(const YAML::Node &parent, std::string key,
               const T &default_value, std::string displayed_name) {
  const YAML::Node value_node{parent[std::string{key}]};

  if (!value_node || value_node.IsNull()) {
    std::cout << displayed_name << " not provided; using default value "
              << '\n';

    return default_value;
  }

  try {
    return value_node.as<T>();
  } catch (const YAML::BadConversion &error) {
    throw std::runtime_error{"Invalid value for '" + std::string{key} +
                             "': " + error.what()};
  }
}