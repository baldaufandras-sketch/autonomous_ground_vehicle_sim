#include "agv_sim/vehicle.hpp"

Vehicle::Vehicle(const VehicleState &initial_state)
    : state_(initial_state), input_{0.0, 0.0} {}

VehicleState Vehicle::getState() const { return state_; }

double Vehicle::getX() const { return state_.x; }

double Vehicle::getY() const { return state_.y; }

double Vehicle::getAcceleration() const { return input_.acceleration; }

double Vehicle::getSpeed() const { return state_.speed; }

double Vehicle::getHeading() const { return state_.heading; }

double Vehicle::getSteeringAngle() const { return input_.steering_angle; }

void Vehicle::setAcceleration(double acceleration) {
  input_.acceleration = acceleration;
}

void Vehicle::setSteeringAngle(double steering_angle) {
  input_.steering_angle = steering_angle;
}

void Vehicle::update(TimeStep dt) {
  state_.speed += dt.seconds * input_.acceleration;
  state_.heading = input_.steering_angle;
}