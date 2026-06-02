#include <gtest/gtest.h>

#include "agv_sim/pure_pursuit_controller.hpp"
#include "agv_sim/vehicle.hpp"
#include <vector>

class PurePursuitControllerTest : public ::testing::Test {
protected:
  double lookahead_distance{0.5};
  PurePursuitController controller{lookahead_distance, 1};

  std::vector<Waypoint> waypoints{
      {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 1.0}, {5.0, 2.0}};

  VehicleState state{0.0, 0.0, 0.0, 1.0};
};

TEST_F(PurePursuitControllerTest, findZeroPoint) {
  state.x = -1;
  state.y = 0;
  controller.computeControl(state, waypoints);
  EXPECT_EQ(controller.getCurrentWaypointIndex(), 0);
}

TEST_F(PurePursuitControllerTest, findNonZeroPoint) {
  state.x = -0.25;
  state.y = 0;
  controller.computeControl(state, waypoints);
  EXPECT_EQ(controller.getCurrentWaypointIndex(), 1);
}

TEST_F(PurePursuitControllerTest, jumpOverPoint) {
  state.x = -0.25;
  state.y = 0;
  double lookAhead = 3;
  controller.setLookaheadDistance(lookAhead);
  controller.computeControl(state, waypoints);
  EXPECT_EQ(controller.getCurrentWaypointIndex(), 3);
}

TEST_F(PurePursuitControllerTest, findLastPoint) {
  state.x = 4.7;
  state.y = 2;
  std::size_t starting_index = 4;
  controller.reset(starting_index);
  controller.computeControl(state, waypoints);
  EXPECT_EQ(controller.getCurrentWaypointIndex(), 4);
}