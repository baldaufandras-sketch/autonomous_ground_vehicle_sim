# Road map

## Current Milestone

### Version 0.6 — Automated Result Evaluation

- Define supported metrics
- Calculate metrics from simulation results
- Process every scenario in a run directory
- Generate and save standard plots
- Store complete effective scenario configuration with the results

## Planned Milestones

### Version 0.7 — Controller Comparison Workflow

- Define a representative benchmark scenario set
- Run equivalent scenarios with different controllers
- Produce aggregate comparison tables and plots
- Define consistent success and failure conditions

### Version 0.8 — Validation and Robustness

- Add parser and metric tests
- Add complete workflow integration tests
- Improve configuration error handling
- Stabilize output formats and units
- Add continuous integration

### Version 1.0 — Complete Portfolio Release

- Finalize README and detailed documentation
- Document architecture, scenarios, metrics, and outputs
- Include representative results
- Verify the default benchmark set
- Clean up the repository for external review

## Completed Milestones


### Version 0.1 - a.k.a "Get it running"

- Set up the repository and build environment
- Implemented the kinematic bicycle model
- Added terminal execution
- Added CSV simulation logging
- Added basic runtime messages

### Version 0.2 – "Hot Pursuit"

- Implemented Pure Pursuit
- Added GoogleTest and CTest
- Added basic validation scenarios
- Added trajectory and steering visualization

### Version 0.3 - "Tie up Loose Ends"

- Added Pure Pursuit end-of-path handling
- Added fallback behavior for reverse-heading situations
- Added plotting and visualization tools and controller diagnostic logging
- Added numerical state validation
- Added steering actuator limits

### Version 0.4 - "Into the Multiverse"

Establish the internal architecture required to simulate different path-tracking controllers and execute multiple scenarios.

- Implemented the Stanley controller
- Introduced a common controller interface
- Added CSV path loading
- Added scenario and controller specification structures
- Added multi-scenario execution
- Added timestamped result directories
- Added per-scenario logs and basic manifests

### Version 0.5 — "Please choose from the menu"

Move scenario configuration out of the source code and establish a practical command-line workflow for running simulations.

- Added a documented YAML scenario format
- Added YAML loading for vehicle, simulation, controller, and path parameters
- Added support for externally referenced path files
- Added validation for required fields, controller types, parameter values, and referenced files
- Added representative scenario files to the repository
- Added command-line execution for single, multiple, and all scenarios
- Added command-line execution to list all available scenario definitions
- Improved configuration  handling and reporting
- Established YAML files as the main user-facing scenario configuration interface