#pragma once
#include "agv_sim/simulation_config.hpp"
#include <cmath>

struct VehicleState {
  double x;
  double y;
  double speed;
  double heading;
};

struct VehicleInput {
  double acceleration;
  double steering_angle;
};

class Vehicle {
public:
    explicit Vehicle(const VehicleState &initial_state, double l_f = 0.3,
                   double l_r = 0.3);

  VehicleState getState() const;

  double getX() const;
  double getY() const;
  double getSpeed() const;
  double getHeading() const;

  double getAcceleration() const;
  double getSteeringAngle() const;

  void setAcceleration(double acceleration);
  void setSteeringAngle(double steering_angle);
  void update(TimeStep dt);

private:
  double l_f_; // COG to front axle
  double l_r_; // COG to rear axle
  VehicleState state_;
  VehicleInput input_;
};