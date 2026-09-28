# Autonomous Navigation & Robotics Engineering Roadmap

This is a hands-on robotics engineering portfolio focused on building the
software foundations required for autonomous mobile robots.

The projects progress from low-level C++ robotics fundamentals to
ROS 2 systems, localization, SLAM, motion planning, navigation,
and autonomous exploration.

The goal is not only to use existing robotics frameworks, but to
understand and implement the fundamental concepts behind them before
integrating production robotics tools.

## Roadmap

| # | Project | Main Topics | Status |
|---|---------|-------------|--------|
| 01 | [Robotics C++ Foundations](./01_robotics_cpp_foundations/) | C++, geometry, occupancy grids, LiDAR, ray tracing, testing | ✅ Complete |
| 02 | ROS 2 Robotics Fundamentals | Nodes, topics, services, actions, QoS, launch | 🔜 Next |
| 03 | Autonomous Mobile Robot Simulation | URDF, TF2, Gazebo, sensors, differential drive | Planned |
| 04 | Localization & Sensor Fusion | Odometry, IMU, EKF, state estimation | Planned |
| 05 | SLAM | Mapping, scan matching, pose estimation | Planned |
| 06 | Motion Planning & Navigation | A*, costmaps, controllers, Nav2 | Planned |
| 07 | Semantic Navigation | Computer vision, perception, semantic information | Planned |
| 08 | Autonomous Exploration | Frontier exploration and integrated autonomy | Planned |

---

## Project 01 — Robotics C++ Foundations

The first project implements fundamental robotics concepts from
scratch in modern C++ before relying on higher-level ROS 2 navigation
libraries.

### Highlights

- 2D vector mathematics
- 2D robot poses and coordinate transformations
- Occupancy-grid representation
- World ↔ grid coordinate conversion
- Bresenham ray tracing
- Simulated 2D LiDAR
- Multi-scan occupancy mapping
- GoogleTest unit testing
- 21 automated tests passing

[View Project 01 →](./01_robotics_cpp_foundations/)

---

## Technology Stack

- C++17
- ROS 2 Jazzy
- Ubuntu 24.04
- CMake
- ament_cmake
- GoogleTest

## Long-Term Goal

Build progressively more complete autonomous navigation systems while
developing practical experience with the algorithms, software
architecture, simulation tools, and ROS 2 ecosystem used in modern
mobile robotics.