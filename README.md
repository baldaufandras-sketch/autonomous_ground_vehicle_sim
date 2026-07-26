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

## Roadmap

The project is currently under active development toward Version 1.0.

See the [detailed development roadmap](docs/roadmap.md).
