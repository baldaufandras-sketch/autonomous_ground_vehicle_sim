#pragma once

#include "vehicle.hpp"
#include "waypoint.hpp"
#include <optional>
#include <vector>

std::optional<Waypoint> findLookaheadPointOnPathSegment(
    const VehicleState &state, const Waypoint &waypoint_1,
    const Waypoint &waypoint_2, const double lookahead_distance);