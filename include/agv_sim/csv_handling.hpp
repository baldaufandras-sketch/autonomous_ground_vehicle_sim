#pragma once
#include "agv_sim/simulation.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>

void writeLogToFile(const SimulationLog &log);
void waypointsToFile(const std::vector<Waypoint> &waypoints);
Path fileToWaypoints(const std::filesystem::path file_location);