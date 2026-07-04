#include "agv_sim/path.hpp"
#include <vector>

Path::Path(const std::vector<Waypoint> &waypoints) : waypoints_(waypoints) {
  buildSegments();
}

const std::vector<Waypoint> &Path::getWaypoints() const { return waypoints_; }
const std::vector<PathSegment> &Path::getSegments() const { return segments_; }
void Path::buildSegments() {
  segments_.clear();

  if (waypoints_.size() < 2) {
    return;
  }

  for (std::size_t i = 0; i + 1 < waypoints_.size(); ++i) {
    segments_.emplace_back(waypoints_[i], waypoints_[i + 1], i);
  }
}