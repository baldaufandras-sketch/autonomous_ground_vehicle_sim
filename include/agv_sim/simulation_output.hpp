#pragma once
#include "agv_sim/simulation.hpp"
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

void writeLogToFile(SimulationLog log,
                    const std::filesystem::path folder_location);
void writeScenarioToYaml(ScenarioRunManifest manifest,
                         const std::filesystem::path folder_location);
void writeResults(const std::vector<ScenarioRunResult> &scenario_list);
void waypointsToFile(const std::vector<Waypoint> &waypoints);
std::string createTimestamp();
Path fileToWaypoints(const std::filesystem::path &folder_location);