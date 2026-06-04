#pragma once
#include "agv_sim/simulation.hpp"
#include <fstream>
#include <iostream>

void writeLogToFile(const SimulationLog &log);
void waypointsToFile(const std::vector<Waypoint> &waypoints);