#pragma once
#include <cmath>
#include <cstddef>
#include <vector>

struct Waypoint {
  double x;
  double y;
};

struct PathSegment {
  const Waypoint start;
  const Waypoint end;
  const double heading;
  const double length;
  const std::size_t index;

  PathSegment(const Waypoint &start_, const Waypoint &end_, std::size_t index_)
      : start(start_), end(end_),
        heading(std::atan2(end.y - start.y, end.x - start.x)),
        length(std::hypot(end.x - start.x, end.y - start.y)), index(index_) {}
};

class Path {
public:
  explicit Path(const std::vector<Waypoint> &waypoints);

  const std::vector<Waypoint> &getWaypoints() const;
  const std::vector<PathSegment> &getSegments() const;

private:
  std::vector<Waypoint> waypoints_;
  std::vector<PathSegment> segments_;
  void buildSegments();
};