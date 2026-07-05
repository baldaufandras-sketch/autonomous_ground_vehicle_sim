# Autonomous Ground Vehicle Simulation

A 2D ground vehicle simulation project written in C++ with Python-based analysis tools.  
The goal is to demonstrate vehicle modeling, path following, controller implementation, data logging, testing, and result visualization. 

## AI Assistance Disclaimer

This project was developed by me as a learning and portfolio project.  
AI tools were used as support for brainstorming, documentation drafting, debugging guidance, and code review suggestions.

All implementation decisions, final code, testing, integration, and validation were performed and reviewed by me.

## Project Goal

This project simulates an autonomous ground vehicle following a predefined waypoint path using a simple vehicle model and controller. Emphasize in this project is for me to learn C++: I plan to go easy on the modeling, and hard on the software environment.

## System Overview

```text
waypoints.csv
     |
     v
+----------------+
| Waypoint Loader |
+----------------+
     |
     v
+----------------+
| Simulation Loop |
+----------------+
     |
     +------------------> Vehicle Model
     |
     +------------------> Controller
     |
     v
trajectory_log.csv
     |
     v
Python Visualization
```

## Road map

Version 0.1 - a.k.a "Get it running"

- Project environment (Git repo, folder structure etc.) is set up
- Executable runs from terminal
- There is a vehicle which moves in a straight line with a predetermined speed
- CSV logfile is generated, which contains time, x, y, heading, speed, steering angle
- Basic informations on the console (sim started, ended, log created etc.)

Version 0.2 – "Hot Pursuit"

- Vehicle tracks a predefined path using a Pure Pursuit controller
- Unit test infrastructure is established using GoogleTest and CTest
- Basic validation scenarios pass, including straight-line, slalom, and circular paths in multiple directions
- Basic visualization tools are available for analyzing simulation results, including reference vs. actual trajectory and steering angle over time
- Limitations can be set on the actuators (rate limiter, minimum and maximum values for e.g. steering angle)

Version 0.3 - "Tie up Loose Ends"

- The Pure Pursuit controller handles the end-of-path condition and brings the vehicle to a stop
- A fallback mode is implemented for situations where the vehicle is oriented away from the path
- Additional plotting and visualization tools are added to support controller analysis and debugging
- Steering actuator limitations are modeled, including maximum steering angle and steering rate limits
- Numerical validity checks are added to detect invalid simulation states

Version 0.4 - "Stanley Scenario"

- Implement the Stanley path-tracking controller as an alternative to the existing Pure Pursuit controller.
- Introduce a common controller interface so that different path-tracking controllers can be selected and exchanged without changing the simulation loop.
- Add a basic scenario selection/editing workflow for running the simulation on different predefined waypoint paths.


