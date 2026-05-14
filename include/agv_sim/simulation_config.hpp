#pragma once
struct TimeStep {
  double seconds;
};

struct SimulationConfig {
  double end_time;
  TimeStep dt;
};