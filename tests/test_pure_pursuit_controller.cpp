#include <gtest/gtest.h>

#include "agv_sim/constants.hpp"
#include "agv_sim/pure_pursuit_controller.hpp"
#include "agv_sim/vehicle.hpp"
#include <vector>

class PurePursuitControllerTest : public ::testing::Test {
protected:
  PurePursuitConfig controllerConfig{
      .lookahead_distance = 1,
      .wheelbase = 1,
      .fallback_steering_angle = constants::pi / 9,
  };
  PurePursuitController controller{controllerConfig};

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

// Lookahead point finder tests
class PurePursuitLookaheadTest : public ::testing::Test {
protected:
  PurePursuitConfig controllerConfig{
      .lookahead_distance = 1,
      .wheelbase = 1,
      .fallback_steering_angle = constants::pi / 9,
  };
  PurePursuitController controller{controllerConfig};
  std::vector<Waypoint> waypoints{
      {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 1.0}, {5.0, 2.0}};

  VehicleState state{0.0, 0.0, 0.0, 1.0};
};

TEST_F(PurePursuitLookaheadTest, alongXAxis) {
  controller.reset(1);
  Waypoint lookahead_point = controller.findLookaheadPoint(state, waypoints);

  EXPECT_NEAR(lookahead_point.x, 0.5, 1e-6);
  EXPECT_NEAR(lookahead_point.y, 0, 1e-6);
}

TEST_F(PurePursuitLookaheadTest, randomPoint) {
  controller.reset(4);
  state.x = 3;
  state.y = 1.5;
  controller.setLookaheadDistance(1);
  Waypoint lookahead_point = controller.findLookaheadPoint(state, waypoints);

  EXPECT_NEAR(lookahead_point.x, 4, 1e-6);
  EXPECT_NEAR(lookahead_point.y, 1.5, 1e-6);
}