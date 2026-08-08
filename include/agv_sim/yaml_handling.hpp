#pragma once

#include "agv_sim/simulation.hpp"
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <yaml-cpp/yaml.h>

Scenario loadScenarioFromYaml(const std::filesystem::path &yaml_path);
template <typename T>
T readOptional(const YAML::Node &parent, std::string key,
               const T &default_value, std::string displayed_name);