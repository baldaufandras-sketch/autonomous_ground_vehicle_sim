#include "agv_sim/vehicle.hpp"

Vehicle::Vehicle(const VehicleState &initial_state, double l_f, double l_r,
                 VehicleLimits limits)
    : state_(initial_state), input_{0.0, 0.0}, l_f_(l_f), l_r_(l_r),
      limits_(limits) {}

VehicleState Vehicle::getState() const { return state_; }

double Vehicle::getX() const { return state_.x; }

double Vehicle::getY() const { return state_.y; }

double Vehicle::getAcceleration() const { return input_.acceleration; }

double Vehicle::getSpeed() const { return state_.speed; }

double Vehicle::getHeading() const { return state_.heading; }

double Vehicle::getSteeringAngle() const {
  return input_.steering_angle_request;
}

VehicleLimits Vehicle::getVehicleLimits() const { return limits_; }

double Vehicle::getWheelbase() const { return (l_f_ + l_r_); }

VehicleState Vehicle::getVehicleState() const { return state_; }

void Vehicle::setAcceleration(double acceleration) {
  input_.acceleration = acceleration;
}

void Vehicle::setInput(VehicleInput controller_request) {
  input_ = controller_request;
}

void Vehicle::update(TimeStep dt) {
  double x_dot{0};
  double y_dot{0};
  double heading_dot{0};
  double beta{0};

  updateSteeringAngle(input_.steering_angle_request, dt);
  state_.speed += dt.seconds * input_.acceleration;
  if (state_.speed < 0) {
    state_.speed = 0;
  }
  beta = atan(l_r_ / (l_r_ + l_f_) * tan(state_.steering_angle));
  x_dot = state_.speed * cos(state_.heading + beta);
  y_dot = state_.speed * sin(state_.heading + beta);
  heading_dot =
      state_.speed / (l_f_ + l_r_) * cos(beta) * tan(state_.steering_angle);

  state_.x += x_dot * dt.seconds;
  state_.y += y_dot * dt.seconds;
  state_.heading += heading_dot * dt.seconds;
}

void Vehicle::updateSteeringAngle(double steering_angle_request, TimeStep dt) {
  state_.steering_angle = std::clamp(steering_angle_request, -0.3, 0.3);
  std::cout << "steering angle updated to: " << state_.steering_angle
            << std::endl;
}