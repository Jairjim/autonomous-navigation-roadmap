# Robotics C++ Foundations

This is a C++ robotics foundations project implementing core mathematical,
mapping, and sensor-simulation concepts from scratch.

## Overview

This project explores fundamental concepts used in autonomous mobile
robotics before relying on higher-level ROS 2 navigation libraries.

The implementation includes 2D geometry, coordinate transformations,
occupancy grids, ray tracing, simulated LiDAR measurements, and
multi-scan occupancy mapping.

## Results

The simulated robot successfully reconstructs an occupancy map from simulated LiDAR measurements and accumulates observations across multiple robot poses.

The image below compares the **ground-truth environment** with the **reconstructed occupancy map** produced by the mapping pipeline.

![Ground Truth vs. Reconstructed Map][def]

The reconstructed map captures the overall structure of the environment; however, some obstacles appear thicker or slightly displaced compared with the ground truth. These artifacts are primarily caused by the simplified fixed-step LiDAR ray-casting implementation and the conversion between continuous world coordinates and discrete grid cells.

These behaviors are intentionally preserved and documented as part of the project's current scope. See [Known Limitations](#known-limitations) for a detailed explanation and planned improvements.
## Features

- 2D vector operations
- 2D robot pose representation
- Angle normalization and conversion
- Local/world coordinate transformations
- 2D occupancy grid representation
- World-to-grid and grid-to-world conversions
- Bresenham grid ray tracing
- Simulated 2D LiDAR measurements
- Occupancy-map updates from laser scans
- Multi-pose scan accumulation
- Automated unit testing with GoogleTest

## Architecture

Ground Truth Environment &rarr; Simulated LiDAR &rarr; LaserScan2D &rarr;
OccupancyGrid2D &rarr;  Reconstructed Map

## Core Components

### Vector2D

Provides fundamental 2D vector operations including magnitude,
normalization, vector arithmetic, and dot products.

### Pose2D

Represents a planar robot pose `(x, y, theta)` and provides coordinate
transformations between local and world reference frames.

### OccupancyGrid2D

Represents a discretized 2D environment containing unknown, free,
and occupied cells.

Supports:

- World/grid coordinate conversion
- Cell access and validation
- Bresenham ray tracing
- Laser-ray insertion
- Laser-scan insertion

### LaserScan2D

Represents a collection of polar LiDAR measurements using angle and
range values together with sensor range limits.

### SimulatedLidar

Generates synthetic LiDAR measurements against a ground-truth
occupancy grid.

## Testing

The project includes automated unit tests for the core mathematical
and mapping components using GoogleTest.

Current test status:

21 tests
0 failures
0 errors

Tests can be executed with:

    colcon test --packages-select robotics_cpp_foundations
    colcon test-result --verbose

## Known Limitations

### LiDAR Ray-Casting Discretization

The current simulated LiDAR uses fixed-step sampling along each laser
beam.

Because the environment is represented by a discrete occupancy grid,
conversions between continuous world coordinates and grid coordinates
can introduce discretization and floating-point boundary effects.

As a result, reconstructed obstacles can occasionally appear one cell
thicker than their ground-truth representation, particularly near cell
boundaries and when using a high angular resolution relative to the
grid resolution.

This limitation belongs primarily to the simplified sensor simulation
rather than the intended occupancy-grid representation.

### Planned Improvement

During the next projects I will replace the fixed-step
sensor ray casting with a grid-aware traversal/intersection approach.

The improved implementation will be compared against this baseline to
demonstrate how the sensor model affects occupancy-map quality.

## Build

From the ROS 2 workspace:

    colcon build \
        --packages-select robotics_cpp_foundations \
        --symlink-install

Then:

    source install/setup.bash

## Run

    ros2 run robotics_cpp_foundations robotics_demo

## Technologies

- C++17
- ROS 2 Jazzy
- CMake
- ament_cmake
- GoogleTest

## Learning Objectives

This project was designed to establish the low-level foundations needed
for later work in:

- Robot localization
- SLAM
- Path planning
- Sensor fusion
- Autonomous navigation
- ROS 2 Nav2

[def]: ./docs/images/mapping_results.jpeg