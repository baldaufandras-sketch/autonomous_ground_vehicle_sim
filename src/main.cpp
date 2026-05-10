#include "agv_sim/vehicle.hpp"
//#include "agv_sim/simulation_config.hpp"
#include <iostream>
using namespace std;

int main()
{
    SimulationConfig config{0.01};
    VehicleState init_state{
    .x = 0.0,
    .y = 5.0,
    .speed = 0.0,
    .heading = 0.0
    };
    Vehicle vehicle{init_state};
    cout << vehicle.getY() << endl;
    return 0;
}