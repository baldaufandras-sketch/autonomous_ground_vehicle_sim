#include <cmath>
#include <gtest/gtest.h>

#include "agv_sim/simulation_config.hpp"
#include "agv_sim/vehicle.hpp"

TEST(KinematicBicycleModelBasicTest, StraightLineZeroSteering) {
  VehicleState initial_state{.x = 0.0, .y = 0.0, .speed = 1.0, .heading = 0.0};

  Vehicle vehicle(initial_state);

  vehicle.setAcceleration(0.0);
  vehicle.setSteeringAngle(0.0);

  TimeStep dt{1.0};

  for (int i = 0; i < 10; ++i) {
    vehicle.update(dt);
  }

  EXPECT_NEAR(vehicle.getX(), 10.0, 1e-9);
  EXPECT_NEAR(vehicle.getY(), 0.0, 1e-9);
  EXPECT_NEAR(vehicle.getSpeed(), 1.0, 1e-9);
  EXPECT_NEAR(vehicle.getHeading(), 0.0, 1e-9);
}

class KinematicBicycleModelTest : public ::testing::Test {
protected:
  static constexpr double speed = 1.0;
  static constexpr double dt_seconds = 1.0;
  static constexpr int steps = 10;
  static constexpr double tolerance = 1e-9;

  void runStraightLineTest(double heading, double speed_value,
                           double expected_x, double expected_y) {
    VehicleState initial_state{0.0, 0.0, speed_value, heading};
    Vehicle vehicle(initial_state);

    vehicle.setAcceleration(0.0);
    vehicle.setSteeringAngle(0.0);

    TimeStep dt{dt_seconds};

    for (int i = 0; i < steps; ++i) {
      vehicle.update(dt);
    }

    EXPECT_NEAR(vehicle.getX(), expected_x, tolerance);
    EXPECT_NEAR(vehicle.getY(), expected_y, tolerance);
  }
};

TEST_F(KinematicBicycleModelTest, MovesPositiveX) {
  runStraightLineTest(0.0, speed, 10.0, 0.0);
}

TEST_F(KinematicBicycleModelTest, MovesPositiveY) {
  runStraightLineTest(M_PI / 2.0, speed, 0.0, 10.0);
}

TEST_F(KinematicBicycleModelTest, MovesNegativeX) {
  runStraightLineTest(M_PI, speed, -10.0, 0.0);
}

TEST_F(KinematicBicycleModelTest, MovesNegativeY) {
  runStraightLineTest(3.0 * M_PI / 2.0, speed, 0.0, -10.0);
}

TEST_F(KinematicBicycleModelTest, MovesInArbitraryDirection45Degrees) {
  const double heading = M_PI / 4.0;
  const double distance = speed * dt_seconds * steps;

  runStraightLineTest(heading, speed, distance * std::cos(heading),
                      distance * std::sin(heading));
}

TEST_F(KinematicBicycleModelTest, MovesInArbitraryDirection30Degrees) {
  const double heading = M_PI / 6.0;
  const double distance = speed * dt_seconds * steps;

  runStraightLineTest(heading, speed, distance * std::cos(heading),
                      distance * std::sin(heading));
}

TEST_F(KinematicBicycleModelTest, MovesInArbitraryDirectionNegativeHeading) {
  const double heading = -M_PI / 3.0;
  const double distance = speed * dt_seconds * steps;

  runStraightLineTest(heading, speed, distance * std::cos(heading),
                      distance * std::sin(heading));
}

TEST_F(KinematicBicycleModelTest, NegativeSpeedMovesOppositeDirection) {
  const double heading = M_PI / 4.0;
  const double negative_speed = -1.0;
  const double distance = negative_speed * dt_seconds * steps;

  runStraightLineTest(heading, negative_speed, distance * std::cos(heading),
                      distance * std::sin(heading));
}