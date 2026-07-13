#include "agv_sim/controller_spec.hpp"

ControllerSpec makePurePursuitSpec(const std::string &name,
                                   const PurePursuitConfig &config) {
  ControllerSpec spec;
  spec.name = name;
  spec.type = ControllerType::PurePursuit;
  spec.pure_pursuit_config = config;
  return spec;
}

ControllerSpec makeStanleySpec(const std::string &name,
                               const StanleyConfig &config) {
  ControllerSpec spec;
  spec.name = name;
  spec.type = ControllerType::Stanley;
  spec.stanley_config = config;
  return spec;
}

std::unique_ptr<IPathTrackingController>
createController(const ControllerSpec &controller_spec) {
  switch (controller_spec.type) {
  case ControllerType::PurePursuit:
    return std::make_unique<PurePursuitController>(
        controller_spec.pure_pursuit_config);

  case ControllerType::Stanley:
    return std::make_unique<StanleyController>(controller_spec.stanley_config);
  }

  throw std::runtime_error("Unknown controller type");
}