# Trajectory Planner Utils 🚀

A practical C++17 library for robot motion profile planning.

`trajectory_planner_utils` provides reusable implementations for:
- Trapezoidal velocity profile (`TVP`)
- Double-S (S-curve) velocity profile (`DSVP`)
- Cartesian straight-line trajectory generation
- Multi-DOF planning via a factory-style interface

It is designed for robotics engineering workflows: easy to integrate, low dependency pressure, and predictable behavior.

## Why this project

- ⚙️ **Engineering-first**: clean interfaces and explicit boundary conditions
- 🎯 **Motion quality options**: choose fast (`TVP`) or smooth (`DSVP`)
- 🧩 **Composable design**: curve components can be reused in custom planners
- 🌍 **Portable**: Linux / macOS / Windows (CMake + C++17)
- 📊 **Visualization-ready**: optional matplotlib-cpp example for quick analysis

## Algorithm At A Glance

| Planner | Best for | Motion smoothness | Complexity |
|---|---|---|---|
| `TVP` (Trapezoidal) | Fast prototyping, simple tasks | Medium (acceleration discontinuity) | Low |
| `DSVP` (Double-S) | Precision motion, vibration-sensitive systems | High (jerk-limited) | Medium |
| `StraightTrajectory` | Cartesian end-effector linear motion | Depends on base profile | Medium |

## Build

### Prerequisites

- CMake >= 3.16
- C++17 compiler (GCC/Clang/MSVC)
- Eigen3 (optional)

### Quick build

```bash
./build.sh
```

Manual build:

```bash
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=ON -DBUILD_TESTS=OFF
cmake --build . -j$(nproc)
```

Install:

```bash
cd build
sudo make install
```

## 60-Second Usage

### 1) Trapezoidal profile (`TVP`)

```cpp
#include <vp/trapezoidal_planner.h>

vp::BCs<double> bc;
bc.start_state.pos = 0.0;
bc.start_state.vel = 0.0;
bc.goal_state.pos  = 1.0;
bc.goal_state.vel  = 0.0;
bc.max_vel         = 0.5;
bc.max_acc         = 0.3;
bc.max_jerk        = 0.0;   // not used by TVP
bc.delta_t         = 0.01;

vp::TrapezoidalPlanner planner({bc}, "TVP");
auto traj = planner.planTrajs();
// point format: [time, pos, vel, acc]
```

### 2) Double-S profile (`DSVP`)

```cpp
#include <vp/double_s_planner.h>

vp::BCs<double> bc;
bc.start_state.pos = 0.0;
bc.start_state.vel = 0.0;
bc.start_state.acc = 0.0;
bc.goal_state.pos  = 1.0;
bc.goal_state.vel  = 0.0;
bc.goal_state.acc  = 0.0;
bc.max_vel         = 0.5;
bc.max_acc         = 0.3;
bc.max_jerk        = 0.5;   // required by DSVP
bc.delta_t         = 0.01;

vp::DoubleSPlanner planner({bc}, "DSVP", 0.95);
auto traj = planner.planTrajs();
// point format: [time, pos, vel, acc, jerk]
```

### 3) Multi-DOF planning

```cpp
#include <vp/multi_velocity_planner.h>

std::vector<vp::BCs<double>> bcs(6);
vp::MultiVelocityPlanner::initDefaultBCs(bcs, 0.5, 0.3, 0.5, 0.01);

vp::MultiVelocityPlanner planner(bcs, "DSVP");
auto traj = planner.getTrajs();
```

### 4) Cartesian straight-line trajectory

```cpp
#include <vp/geometry_trajectory/straight_trajectory.h>

std::vector<double> start_pose = {0.0, 0.0, 0.5, 0.0, 0.0, 0.0};
std::vector<double> goal_pose  = {0.5, 0.3, 0.8, 0.0, 0.0, 1.57};

vp::BCs<double> bc;
bc.start_state.pos = 0.0;
bc.goal_state.pos  = 1.0;
bc.max_vel         = 0.5;
bc.max_acc         = 0.3;
bc.max_jerk        = 0.5;
bc.delta_t         = 0.01;

vp::StraightTrajectory straight(start_pose, goal_pose, {bc}, "DSVP");
auto cart_traj = straight.getTrajs();
// sample format: [time, x, y, z, rx, ry, rz, ...]
```

## Exceptions & Error Codes

```cpp
try {
    vp::TrapezoidalPlanner planner({bc}, "TVP");
    auto traj = planner.planTrajs();
} catch (const vp::PlannerException& e) {
    std::cerr << "Error " << e.errorCode() << ": " << e.what() << std::endl;
}
```

Common error codes:
- `1001`: constraints missing
- `1002`: calculated pose is empty
- `1008`: value out of boundary
- `1009`: empty input value

## Run Examples

```bash
cd build/examples
./example_trapezoidal
./example_double_s
./example_straight_trajectory
./example_comparison
./example_multi_dof
```

## Visualization (Optional) 📊

`example_visualization` is enabled when Python3 dev + NumPy are found.

Dependencies (Ubuntu/Debian):

```bash
sudo apt install python3-dev python3-numpy python3-matplotlib
```

Run:

```bash
cd build/examples
./example_visualization
```

Generated images:
- `velocity_comparison.png`
- `double_s_kinematics.png`
- `cartesian_trajectory.png`
- `acceleration_comparison.png`

More details: [VISUALIZATION_GUIDE.md](VISUALIZATION_GUIDE.md)

## Project Structure

```text
trajectory_planner_utils/
├── include/vp/
│   ├── velocity_planner_interface.h
│   ├── planner_exception.h
│   ├── trapezoidal_planner.h
│   ├── double_s_planner.h
│   ├── multi_velocity_planner.h
│   ├── velocity_planning.h
│   ├── curve_interface/
│   └── geometry_trajectory/
├── include/third_party/
│   └── matplotlibcpp.h
├── src/
├── examples/
├── CMakeLists.txt
├── build.sh
└── *.md
```

## Docs Map

- [QUICKSTART.md](QUICKSTART.md): 5-minute onboarding
- [QUICK_REFERENCE.md](QUICK_REFERENCE.md): API and decision cheat sheet
- [PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md): architecture and module-level perspective
- [VISUALIZATION_GUIDE.md](VISUALIZATION_GUIDE.md): plotting workflow and troubleshooting

## Roadmap

- [ ] Add unit tests (GoogleTest)
- [ ] Add CI checks (build + examples)
- [ ] Add Python bindings (pybind11)
- [ ] Add ROS/ROS2 examples
- [ ] Add benchmark suite

## Contributing

Issues and PRs are welcome.

If you want to contribute a new planner, keep it aligned with the existing `VelocityPlannerInterface` and provide:
- clear constraints definition
- deterministic output format
- at least one example program

## License

Apache License 2.0

---

Build trajectories with fewer surprises, and debug them with plots before your robot finds the surprises for you 🤖✨
