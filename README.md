# Trajectory Planner Utils 🚀

一个面向工程落地的 C++17 轨迹规划库。  
目标很明确：**依赖少、集成快、行为可预测**。

这个仓库聚焦把可复用的速度规划与几何轨迹模块统一到一个库中。

## 为什么用它 🧩

- ⚡ **快速集成**：一个主库 `velocity_planning`，直接链接即可
- 🎯 **算法完整**：`TVP`（梯形）、`DSVP`（Double-S）、直线/圆弧/SAS/CL 等轨迹模块
- 🛠️ **工程友好**：支持按 CMake 开关裁剪模块，尽量减少外部依赖
- ✅ **可执行验证**：提供 staged examples 和一键测试脚本

## 一体化架构（重点）

统一库：`velocity_planning`

包含模块：
- Core 速度规划：`TVP` / `DSVP` / Multi-DOF / StraightTrajectory
- 兼容层速度规划：`VelocityPlannerCompat`（`vp::tp` 命名空间）
- 几何轨迹：Straight / Circle / Ellipse / StraightArcTransition
- SAS 多点轨迹：`MultiPointsTrajectory` / `SASTrajectory`
- CL 轨迹：`NormalizedCircleLineTrajPlanner` / `SplicedCircleLineTrajPlanner` / `CircleLineTrajGeneration`

统一入口头文件：

```cpp
#include <vp/trajectory_planning.h>
```

## 依赖与裁剪 🔧

基础依赖：
- CMake >= 3.16
- C++17 编译器
- Eigen3（推荐）

可选依赖：
- `yaml-cpp`：CL / 部分拓展模块需要
- Python3 dev + NumPy：可视化示例需要

常用 CMake 开关：
- `-DBUILD_UNIFIED_TRAJECTORY_MODULES=ON`
- `-DBUILD_UNIFIED_CL_MODULES=ON/OFF`
- `-DBUILD_UNIFIED_SAS_MODULES=ON/OFF`
- `-DBUILD_UNIFIED_TOTG_ADAPTER=ON/OFF`
- `-DBUILD_UNIFIED_PLOTTING=ON/OFF`

## 快速开始（推荐）

```bash
./build.sh
```

或手动构建：

```bash
mkdir -p build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=ON \
  -DBUILD_TESTS=OFF \
  -DBUILD_UNIFIED_TRAJECTORY_MODULES=ON \
  -DBUILD_UNIFIED_CL_MODULES=ON \
  -DBUILD_UNIFIED_SAS_MODULES=ON
cmake --build . -j$(nproc)
```

## 一分钟验证 ✅

```bash
./test_all.sh
```

脚本会自动跑：
- 基础速度规划 examples
- staged unified examples（stage1/2/3）
- 可视化示例（若环境满足）

## 示例路线图（从 core 到完整轨迹）

```bash
cd build/examples
./example_stage1_unified_core
./example_stage2_unified_sas
./example_stage3_unified_cl
```

含义：
- `stage1`：统一 core（速度规划 + 基础直线轨迹）
- `stage2`：SAS 多点轨迹
- `stage3`：CL 轨迹（normalized / spliced / generation）

## 最小集成示例

```cpp
#include <vp/trajectory_planning.h>

int main() {
    std::vector<vp::tp::BCs<double>> bcs(1);
    vp::tp::initDefaultBCs(bcs, 0.4, 0.3, 0.5, 0.01);
    bcs[0].s_state.pos = 0.0;
    bcs[0].g_state.pos = 1.0;

    vp::tp::VelocityPlannerCompat planner(bcs, "DSVP");
    auto raw = planner.getTrajs(false);
    return raw.empty() ? 1 : 0;
}
```

## 输出数据约定 📐

常见轨迹点格式：
- 速度轨迹：`[time, pos, vel, acc, jerk]`
- 6D 轨迹：`[time, pose(6), vel(6), acc(6)]`

## 项目结构

```text
trajectory_planner_utils/
├── include/vp/
│   ├── trajectory_planning.h
│   ├── velocity_planning.h
│   ├── common/
│   ├── trajectory_plan/
│   ├── geometry_trajectory/
│   └── curve_interface/
├── src/
├── examples/
├── build.sh
├── test_all.sh
└── README.md
```

## 文档索引

- [QUICKSTART.md](QUICKSTART.md)
- [QUICK_REFERENCE.md](QUICK_REFERENCE.md)
- [PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md)
- [VISUALIZATION_GUIDE.md](VISUALIZATION_GUIDE.md)

## Roadmap

- [ ] 增加 GTest 单测覆盖（重点覆盖 unified modules）
- [ ] 增加 CI（构建 + staged examples）
- [ ] 增加 benchmark 与性能回归

## License

Apache License 2.0

---

让轨迹规划更像工程系统，而不只是一个“能跑起来”的 demo 🤖
