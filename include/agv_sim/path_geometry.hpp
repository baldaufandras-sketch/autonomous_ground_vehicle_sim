#pragma once

#include "path.hpp"
#include "vehicle.hpp"
#include <optional>
#include <vector>

std::optional<Waypoint> findLookaheadPointOnPathSegment(
    const VehicleState &state, const Waypoint &waypoint_1,
    const Waypoint &waypoint_2, const double lookahead_distance);

Waypoint chooseSolution(const Waypoint wp_prev, const Waypoint wp_curr,
                        const Waypoint solution_1, const Waypoint solution_2);

double dotBetweenSegments(const Waypoint &p1, const Waypoint &p2,
                          const Waypoint &q1, const Waypoint &q2);