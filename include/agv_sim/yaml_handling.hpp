#pragma once

#include "agv_sim/simulation.hpp"
#include <filesystem>
#include <yaml-cpp/yaml.h>

Scenario loadScenarioFromYaml(const std::filesystem::path &yaml_path);