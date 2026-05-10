#pragma once
#include "agv_sim/simulation_config.hpp"
struct VehicleState
{
    double x;
    double y;
    double speed;
    double heading;
};

struct VehicleInput
{
    double acceleration;
    double steering_angle;
};

class Vehicle
{
public:
    explicit Vehicle(const VehicleState& initial_state);

    VehicleState getState() const;

    double getX() const;
    double getY() const;
    double getSpeed() const;
    double getHeading() const;

    double getAcceleration() const;
    double getSteeringAngle() const;

    void setAcceleration(double acceleration);
    void setSteeringAngle(double steering_angle);
    void update(double dt);

private:
    VehicleState state_;
    VehicleInput input_;
};