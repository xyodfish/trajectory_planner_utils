# Project Overview: `trajectory_planner_utils` 🧭

`trajectory_planner_utils` is a focused C++17 motion-planning utility library.

Repository name: `trajectory_planner_utils`  
CMake package/library name: `velocity_planning`

This document explains what the project solves, how modules are organized, and how to evolve it safely.

## 1. Project Positioning

This project targets developers who need reliable motion profile planning blocks without pulling in a full robotics framework.

Typical use cases:
- industrial manipulator path execution
- motion control prototyping
- research on velocity profile quality (smoothness vs speed)
- educational demonstrations of TVP/DSVP behavior

## 2. Core Capabilities

### Velocity Planning

- **Trapezoidal Velocity Profile (TVP)**
  - 3-phase profile: accel -> cruise -> decel
  - low compute overhead
  - good baseline for many tasks

- **Double-S Velocity Profile (DSVP)**
  - 7-phase jerk-limited profile
  - smoother acceleration transitions
  - better for vibration-sensitive motion

### Geometry Trajectory

- **Cartesian StraightTrajectory**
  - interpolates `[x, y, z, rx, ry, rz]`
  - plugs into TVP or DSVP timing
  - useful for end-effector linear motion workflows

### Multi-DOF Planning

- **MultiVelocityPlanner**
  - factory-style planner selection (`"TVP"` / `"DSVP"`)
  - unified boundary-condition initialization for multi-axis tasks

## 3. Architecture Snapshot

```text
include/vp/
├── velocity_planner_interface.h    # abstract interfaces + core data types
├── trapezoidal_planner.h           # TVP planner
├── double_s_planner.h              # DSVP planner
├── multi_velocity_planner.h        # planner factory facade
├── geometry_trajectory/
│   └── straight_trajectory.h       # Cartesian straight line trajectory
└── curve_interface/
    ├── curve_interface.h
    ├── constant_jerk_trajectory.h
    └── piecewise_trajectory.h

src/
├── trapezoidal/
├── double_s/
├── geometry_trajectory/
├── curve_interface/
└── multi_velocity_planner.cpp
```

Design intent:
- keep planner APIs explicit
- isolate algorithm math from application-level control loops
- preserve module boundaries for extension

## 4. Public API Model

Main public types:
- `vp::KinematicState<T>`
- `vp::BoundaryConditions<T>` / alias `vp::BCs<T>`
- `vp::VelocityPlannerInterface<T>`
- `vp::TrapezoidalPlanner`
- `vp::DoubleSPlanner`
- `vp::MultiVelocityPlanner`
- `vp::StraightTrajectory`

Error strategy:
- exceptions via `vp::PlannerException`
- numeric error codes for structured handling

## 5. Build & Dependency Policy

Required:
- C++17 compiler
- CMake >= 3.16

Optional:
- Eigen3
- Python3 + NumPy + Matplotlib (visualization example only)

Non-goals:
- no hard dependency on ROS/ROS2
- no hardware-driver coupling

## 6. Engineering Quality Notes

What is already good:
- clear separation between headers and implementation
- multiple runnable examples
- straightforward CMake setup

What should be strengthened next:
- automated unit tests (GoogleTest)
- CI pipeline for build + examples
- benchmark harness for planner performance/quality metrics

## 7. Recommended Evolution Path

1. Add test coverage for boundary conditions and degenerate inputs.
2. Introduce benchmark cases (short move, long move, high jerk limit, low jerk limit).
3. Add extension guide for implementing new planners against `VelocityPlannerInterface`.
4. Optionally add language bindings (pybind11) for fast experimentation.

## 8. Document Map

- Main guide: [README.md](README.md)
- Fast onboarding: [QUICKSTART.md](QUICKSTART.md)
- Cheat sheet: [QUICK_REFERENCE.md](QUICK_REFERENCE.md)
- Visualization workflow: [VISUALIZATION_GUIDE.md](VISUALIZATION_GUIDE.md)

## 9. TL;DR

If you need a small, practical motion-planning library with both classic and jerk-limited profiles, this project gives you solid building blocks without framework lock-in.  
Professional enough for integration, lightweight enough for fast iteration ⚙️
