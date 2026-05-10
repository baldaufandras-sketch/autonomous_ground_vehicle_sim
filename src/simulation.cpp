#include "simulation.hpp"
#include "vehicle.hpp"

SimulationLog runSimulation(
    Vehicle vehicle,
    const std::vector<Waypoint>& waypoints,
    const SimulationConfig& config
);