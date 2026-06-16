#pragma once

#include "vehicle.hpp"
#include "waypoint.hpp"
#include <optional>
#include <vector>

std::optional<Waypoint> findLookaheadPointOnPathSegment(
    const VehicleState &state, const Waypoint &waypoint_1,
    const Waypoint &waypoint_2, const double lookahead_distance);

Waypoint chooseSolution(const Waypoint wp_prev, const Waypoint wp_curr,
                        const Waypoint solution_1, const Waypoint solution_2);