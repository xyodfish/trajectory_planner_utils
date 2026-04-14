# Velocity Planning Library

A lightweight, header-friendly C++ library for robot velocity profile planning. This library provides implementations of common velocity planning algorithms used in robotics and motion control.

## Features

- ✅ **Trapezoidal Velocity Profile** - Classic acceleration-cruise-deceleration profile
- ✅ **Double-S (S-Curve) Profile** - Smooth jerk-limited trajectory with 7 phases
- ✅ **Cartesian Straight Line Trajectory** - Linear motion in Cartesian space with pose interpolation
- ✅ **Multi-DOF Support** - Plan trajectories for multiple degrees of freedom
- ✅ **Factory Pattern** - Easy to extend with new planning algorithms
- ✅ **Curve Interface** - Modular design with ConstantJerk and PiecewiseTrajectory
- ✅ **Header-only Friendly** - Minimal dependencies (only Eigen3 optional)
- ✅ **Cross-platform** - Works on Linux, Windows, macOS

## Quick Start

### Prerequisites

- CMake >= 3.16
- C++17 compatible compiler
- Eigen3 (optional, recommended)

### Build & Install

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

### Basic Usage

#### Example 1: Single DOF Trapezoidal Planning

```cpp
#include <vp/trapezoidal_planner.h>

// Define boundary conditions
vp::BCs<double> bc;
bc.start_state.pos = 0.0;
bc.start_state.vel = 0.0;
bc.goal_state.pos  = 1.0;
bc.goal_state.vel  = 0.0;
bc.max_vel         = 0.5;   // m/s
bc.max_acc         = 0.3;   // m/s²
bc.delta_t         = 0.01;  // 10ms

// Create planner and generate trajectory
vp::TrapezoidalPlanner planner({bc}, "TVP");
auto trajectory = planner.planTrajs();

// Access trajectory points
for (const auto& point : trajectory) {
    double time     = point[0];  // seconds
    double position = point[1];  // normalized position
    double velocity = point[2];  // m/s
}
```

#### Example 2: Double-S (S-Curve) Planning

```cpp
#include <vp/double_s_planner.h>

// Define boundary conditions with jerk limit
vp::BCs<double> bc;
bc.start_state.pos = 0.0;
bc.start_state.vel = 0.0;
bc.goal_state.pos  = 1.0;
bc.goal_state.vel  = 0.0;
bc.max_vel         = 0.5;   // m/s
bc.max_acc         = 0.3;   // m/s²
bc.max_jerk        = 0.5;   // m/s³ (important for S-curve!)
bc.delta_t         = 0.01;

// Create Double-S planner with gamma parameter (time scaling)
vp::DoubleSPlanner planner({bc}, "DSVP", 0.95);
auto trajectory = planner.planTrajs();

// Access full kinematic states [time, pos, vel, acc, jerk]
for (const auto& point : trajectory) {
    double time     = point[0];
    double position = point[1];
    double velocity = point[2];
    double accel    = point[3];
    double jerk     = point[4];
}
```

#### Example 3: Cartesian Space Straight Line Trajectory

```cpp
#include <vp/geometry_trajectory/straight_trajectory.h>

// Define start and goal poses [x, y, z, rx, ry, rz]
std::vector<double> start_pose = {0.0, 0.0, 0.5, 0.0, 0.0, 0.0};
std::vector<double> goal_pose  = {0.5, 0.3, 0.8, 0.0, 0.0, 1.57};

// Define boundary conditions for normalized time
vp::BCs<double> bc;
bc.start_state.pos = 0.0;
bc.goal_state.pos  = 1.0;  // Normalized
bc.max_vel         = 0.5;
bc.max_acc         = 0.3;
bc.max_jerk        = 0.5;
bc.delta_t         = 0.01;

// Create straight trajectory with S-curve profile
vp::StraightTrajectory straight_traj(start_pose, goal_pose, {bc}, "DSVP");
auto traj = straight_traj.getTrajs();

// Access Cartesian trajectory
// Format: [time, x, y, z, rx, ry, rz, vx, vy, vz, wx, wy, wz, ax, ay, az, ...]
for (const auto& point : traj) {
    double time = point[0];
    double x = point[1], y = point[2], z = point[3];
    double rx = point[4], ry = point[5], rz = point[6];
    // ... velocities and accelerations follow
}
```

#### Example 4: Multi-DOF Planning with Factory Pattern

```cpp
#include <vp/multi_velocity_planner.h>

// Define 6-DOF boundary conditions
std::vector<vp::BCs<double>> bcs(6);
vp::MultiVelocityPlanner::initDefaultBCs(bcs, 0.5, 0.3, 0.5, 0.01);

// Create multi-DOF planner - choose algorithm
vp::MultiVelocityPlanner planner(bcs, "TVP");   // Trapezoidal
// or
vp::MultiVelocityPlanner planner(bcs, "DSVP");  // Double-S

auto trajectory = planner.getTrajs();
```

## API Reference

### Core Interfaces

#### `vp::KinematicState<T>`
Represents the kinematic state at a specific time.

```cpp
template <typename T>
struct KinematicState {
    T time;   // Time stamp
    T pos;    // Position
    T vel;    // Velocity
    T acc;    // Acceleration
    T jerk;   // Jerk (3rd derivative)
};
```

#### `vp::BoundaryConditions<T>`
Defines start/goal states and constraints.

```cpp
template <typename T>
struct BoundaryConditions {
    KinematicState<T> start_state;
    KinematicState<T> goal_state;
    T max_vel;
    T max_acc;
    T max_jerk;
    T delta_t;
};
```

### Planners

#### `vp::TrapezoidalPlanner`
Classic trapezoidal velocity profile with three phases:
1. Constant acceleration
2. Constant velocity (optional)
3. Constant deceleration

**Pros**: Simple, fast computation, time-optimal  
**Cons**: Discontinuous acceleration (infinite jerk), may cause vibration

#### `vp::DoubleSPlanner`
Smooth S-curve velocity profile with seven phases:
1. Increasing acceleration (positive jerk)
2. Constant acceleration
3. Decreasing acceleration (negative jerk)
4. Constant velocity
5. Increasing deceleration (negative jerk)
6. Constant deceleration
7. Decreasing deceleration (positive jerk)

**Pros**: Continuous acceleration, limited jerk, smooth motion  
**Cons**: More complex, slightly longer trajectory time

**Constructor Parameters**:
- `BCs`: Boundary conditions
- `name`: Algorithm identifier
- `gamma`: Time scaling factor (0.8-1.0, default 0.95)

#### `vp::StraightTrajectory`
Cartesian space straight line trajectory planner that combines:
- **Linear position interpolation** between start and goal poses
- **Orientation interpolation** (Euler angles or quaternions)
- **Velocity profile planning** using TVP or DSVP

**Key Features**:
- Generates smooth Cartesian paths for end-effector motion
- Supports both joint-space and Cartesian-space planning modes
- Calculates velocities and accelerations via numerical differentiation
- Returns full 6-DOF pose trajectory [x, y, z, rx, ry, rz]

**Use Cases**:
- Linear welding/sealing paths
- Pick-and-place operations
- Precision assembly tasks
- Surface following applications

**Constructor Parameters**:
- `start_pose`: Start pose [x, y, z, rx, ry, rz]
- `goal_pose`: Goal pose
- `BCs`: Boundary conditions for normalized time
- `alg`: Velocity profile algorithm ("TVP" or "DSVP")

#### `vp::MultiVelocityPlanner`
Factory-based planner supporting multiple algorithms:
- `"TVP"` - Trapezoidal Velocity Profile
- `"DSVP"` - Double-S Velocity Profile

### Curve Interface (Advanced)

The library includes modular curve components:

- **`vp::CurveInterface`** - Abstract base for all curve types
- **`vp::ConstantJerkTrajectory`** - Single segment with constant jerk
- **`vp::PiecewiseTrajectory`** - Composite trajectory from multiple segments

These can be used to build custom trajectory planners.

### Exception Handling

```cpp
try {
    // Planning code
} catch (const vp::PlannerException& e) {
    std::cerr << "Error " << e.errorCode() << ": " << e.what() << std::endl;
}
```

Error codes:
- `1001` - Constraints not given
- `1002` - Calculated pose is empty
- `1008` - Value extends boundary
- `1009` - Empty input value

## Examples

Build examples:
```bash
cmake .. -DBUILD_EXAMPLES=ON
make
```

Run examples:
```bash
./examples/example_trapezoidal           # Basic trapezoidal planning
./examples/example_double_s              # Double-S (S-Curve) planning
./examples/example_straight_trajectory   # Cartesian straight line trajectory
./examples/example_comparison            # Compare both algorithms
./examples/example_multi_dof             # Multi-DOF planning
```

## Project Structure

```
velocity_planning/
├── include/vp/              # Public headers
│   ├── velocity_planner_interface.h
│   ├── planner_exception.h
│   ├── trapezoidal_planner.h
│   ├── double_s_planner.h
│   ├── multi_velocity_planner.h
│   ├── velocity_planning.h
│   └── curve_interface/     # Advanced curve components
│   │   ├── curve_interface.h
│   │   ├── constant_jerk_trajectory.h
│   │   └── piecewise_trajectory.h
│   └── geometry_trajectory/ # Geometry trajectory planners
│       └── straight_trajectory.h
├── src/                     # Implementation files
│   ├── curve_interface/
│   ├── trapezoidal/
│   ├── double_s/
│   ├── geometry_trajectory/
│   │   └── straight_trajectory.cpp
│   └── multi_velocity_planner.cpp
├── examples/                # Example programs
├── tests/                   # Unit tests (coming soon)
├── CMakeLists.txt
└── README.md
```

## Design Principles

1. **Minimal Dependencies**: Only requires C++17 standard library
2. **Template-based**: Supports different numeric types (float, double, etc.)
3. **Exception Safety**: Clear error reporting via exceptions
4. **Extensible**: Easy to add new planning algorithms
5. **No Hardware Coupling**: Pure algorithmic implementation
6. **Modular Architecture**: Reusable curve components

## Algorithm Comparison

| Feature | Trapezoidal | Double-S |
|---------|-------------|----------|
| Phases | 3 | 7 |
| Jerk | Infinite (discontinuous) | Limited (continuous) |
| Smoothness | Low | High |
| Computation | Fast | Moderate |
| Trajectory Time | Shorter | Slightly longer |
| Vibration | Higher | Lower |
| Use Case | Simple motions | Precision tasks |

## Roadmap

- [x] Complete Double-S (S-Curve) planner implementation
- [x] Curve interface architecture
- [ ] Add unit tests with Google Test
- [ ] Python bindings via pybind11
- [ ] ROS/ROS2 integration examples
- [ ] Performance benchmarks
- [ ] Additional planners (MinSnap, TOPP-RA, etc.)
- [ ] Visualization tools

## License

Apache License 2.0

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues.

## Acknowledgments

This library was extracted from industrial robot control systems and refined for general-purpose use. The Double-S implementation uses constant jerk trajectory segments拼接 to achieve smooth motion profiles.

Special thanks to the robotics community for inspiring these implementations.