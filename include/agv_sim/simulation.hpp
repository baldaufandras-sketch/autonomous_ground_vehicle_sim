#pragma once

#include "agv_sim/vehicle.hpp"
#include "agv_sim/waypoint.hpp"
#include "agv_sim/simulation_config.hpp"

#include <vector>

struct SimulationSample
{
    double time;
    double x;
    double y;
    double speed;
    double heading;
    double acceleration;
    double steering_angle;
};

using SimulationLog = std::vector<SimulationSample>;

SimulationLog runSimulation(
    Vehicle vehicle,
    const std::vector<Waypoint>& waypoints,
    const SimulationConfig& config
);