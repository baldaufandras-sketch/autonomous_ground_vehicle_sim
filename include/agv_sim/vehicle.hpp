#pragma once
#include "agv_sim/constants.hpp"
#include "agv_sim/simulation_config.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

struct VehicleState {
  double x;
  double y;
  double speed;
  double heading;
  double steering_angle;
};

struct VehicleInput {
  double acceleration;
  double steering_angle_request;
};

struct VehicleLimits {
  double maxSteeringAngle;
  double maxSteeringAngleGradient; // in radians per second
};

class Vehicle {
public:
  explicit Vehicle(const VehicleState &initial_state, double l_f = 0.3,
                   double l_r = 0.3,
                   VehicleLimits limits = {constants::pi / 6.0,
                                           constants::pi / 18});

  VehicleState getState() const;

  double getX() const;
  double getY() const;
  double getSpeed() const;
  double getHeading() const;

  double getAcceleration() const;
  double getSteeringAngle() const;
  double getWheelbase() const;
  VehicleLimits getVehicleLimits() const;
  VehicleState getVehicleState() const;

  void setAcceleration(double acceleration);
  void setInput(VehicleInput controller_request);
  void update(TimeStep dt);

private:
  double l_f_; // COG to front axle
  double l_r_; // COG to rear axle
  void updateSteeringAngle(double steering_angle_request, TimeStep dt);

  VehicleState state_;
  VehicleInput input_;
  VehicleLimits limits_;
};