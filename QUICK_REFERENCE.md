# Quick Reference Card 🧠

面向“写代码时随手查”的速查卡。

## Build & Run

```bash
./build.sh
cd build/examples
./example_trapezoidal
./example_double_s
./example_multi_dof
./example_straight_trajectory
./example_comparison
./example_stage1_unified_core
./example_stage2_unified_sas
./example_stage3_unified_cl
```

可视化示例：

```bash
./example_visualization
```

## 最常用数据结构

```cpp
template <typename T>
struct KinematicState {
    T time;
    T pos;
    T vel;
    T acc;
    T jerk;
    T theta;
};

template <typename T>
struct BoundaryConditions {
    KinematicState<T> start_state;
    KinematicState<T> goal_state;
    T max_acc;
    T max_vel;
    T max_jerk;
    T delta_t;
};
```

别名：`vp::BCs<T> = vp::BoundaryConditions<T>`

统一头文件（核心 + 扩展轨迹模块）：

```cpp
#include <vp/trajectory_planning.h>
```

## 一眼选算法

| 场景 | 推荐 | 说明 |
|---|---|---|
| 快速验证 | `TVP` | 实现简单，计算开销低 |
| 精密装配 | `DSVP` | jerk 受限，更平滑 |
| 末端线性轨迹 | `StraightTrajectory` + `DSVP` | 路径直、速度平滑 |
| 多轴任务 | `MultiVelocityPlanner` | 统一接口管理多 DOF |

## 关键 API

```cpp
// Single planner
auto traj = planner.planTrajs(isNormalized);
auto states = planner.planKStates(isNormalized);
auto kstate = planner.getKState(time, isNormalized);
auto end = planner.getEndTraj(isNormalized);

// Multi-DOF planner
auto multiTraj = multiPlanner.getTrajs(isNormalized);
```

## 典型代码片段

### TVP

```cpp
#include <vp/trapezoidal_planner.h>

vp::BCs<double> bc;
bc.start_state.pos = 0.0;
bc.goal_state.pos  = 1.0;
bc.max_vel         = 0.5;
bc.max_acc         = 0.3;
bc.max_jerk        = 0.0;
bc.delta_t         = 0.01;

vp::TrapezoidalPlanner planner({bc}, "TVP");
auto traj = planner.planTrajs();
```

### DSVP

```cpp
#include <vp/double_s_planner.h>

vp::BCs<double> bc;
bc.start_state.pos = 0.0;
bc.goal_state.pos  = 1.0;
bc.max_vel         = 0.5;
bc.max_acc         = 0.3;
bc.max_jerk        = 0.5;   // 必填
bc.delta_t         = 0.01;

vp::DoubleSPlanner planner({bc}, "DSVP", 0.95);
auto traj = planner.planTrajs();
```

### Multi-DOF

```cpp
#include <vp/multi_velocity_planner.h>

std::vector<vp::BCs<double>> bcs(6);
vp::MultiVelocityPlanner::initDefaultBCs(bcs, 0.5, 0.3, 0.5, 0.01);

vp::MultiVelocityPlanner planner(bcs, "TVP");
auto traj = planner.getTrajs();
```

## 常见错误码

| Code | 含义 |
|---|---|
| `1001` | Constraints are not given |
| `1002` | Calculated pose is empty |
| `1008` | Value extends boundary |
| `1009` | Empty input value |

## 高频排障

- `DSVP` 结果异常：先检查 `max_jerk` 是否设置
- 点数太多：增大 `delta_t`（例如从 `0.005` 调到 `0.01`）
- 轨迹太“硬”：改用 `DSVP` 或降低 `max_acc`
- 轨迹时间不对：确认 `start_state` 与 `goal_state` 不相同

## 文档入口

- 主文档：[README.md](README.md)
- 快速上手：[QUICKSTART.md](QUICKSTART.md)
- 架构说明：[PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md)
- 可视化指南：[VISUALIZATION_GUIDE.md](VISUALIZATION_GUIDE.md)

Happy planning, fewer surprises 🤖
