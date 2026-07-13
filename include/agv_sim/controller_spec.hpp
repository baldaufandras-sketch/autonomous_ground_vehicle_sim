#include "agv_sim/pure_pursuit_controller.hpp"
#include "agv_sim/stanley_controller.hpp"
#include <memory>

enum class ControllerType { PurePursuit, Stanley };

struct ControllerSpec {
  std::string name;
  ControllerType type;

  PurePursuitConfig pure_pursuit_config{};
  StanleyConfig stanley_config{};
};

ControllerSpec
makePurePursuitSpec(const std::string &name,
                    const PurePursuitConfig &config = PurePursuitConfig{});

ControllerSpec makeStanleySpec(const std::string &name,
                               const StanleyConfig &config = StanleyConfig{});

std::unique_ptr<IPathTrackingController>
createController(const ControllerSpec &controller_spec);
