# Quick Start Guide

## 5分钟快速上手 Velocity Planning Library

### 1. 构建库

```bash
cd velocity_planning
./build.sh
```

### 2. 运行示例

```bash
cd build
./examples/example_trapezoidal
./examples/example_multi_dof
```

### 3. 在你的项目中使用

#### CMakeLists.txt
```cmake
find_package(velocity_planning REQUIRED)

add_executable(my_app main.cpp)
target_link_libraries(my_app velocity_planning)
```

#### main.cpp - 简单示例
```cpp
#include <vp/velocity_planning.h>
#include <iostream>

int main() {
    // 定义边界条件
    vp::BCs<double> bc;
    bc.start_state.pos = 0.0;
    bc.start_state.vel = 0.0;
    bc.goal_state.pos  = 1.0;
    bc.goal_state.vel  = 0.0;
    bc.max_vel         = 0.5;
    bc.max_acc         = 0.3;
    bc.delta_t         = 0.01;
    
    // 创建规划器并生成轨迹
    vp::TrapezoidalPlanner planner({bc}, "TVP");
    auto trajectory = planner.planTrajs();
    
    std::cout << "Generated " << trajectory.size() << " points" << std::endl;
    
    return 0;
}
```

### 4. 多自由度规划

```cpp
#include <vp/multi_velocity_planner.h>

// 6轴机器人
std::vector<vp::BCs<double>> bcs(6);
vp::MultiVelocityPlanner::initDefaultBCs(bcs, 0.5, 0.3, 0.0, 0.01);

// 设置目标位置
bcs[0].goal_state.pos = 0.5;
bcs[1].goal_state.pos = -0.3;
// ... 其他轴

vp::MultiVelocityPlanner planner(bcs, "TVP");
auto trajectory = planner.getTrajs();
```

### 5. 错误处理

```cpp
try {
    vp::TrapezoidalPlanner planner({bc}, "TVP");
    auto traj = planner.planTrajs();
} catch (const vp::PlannerException& e) {
    std::cerr << "Error " << e.errorCode() << ": " << e.what() << std::endl;
}
```

### 常见错误码

| 代码 | 说明 |
|------|------|
| 1001 | 未提供约束条件 |
| 1002 | 计算的位置为空 |
| 1008 | 值超出边界 |
| 1009 | 输入值为空 |

## API 速查

### 核心类

- **`vp::KinematicState<T>`** - 运动学状态（时间、位置、速度、加速度、加加速度）
- **`vp::BoundaryConditions<T>`** - 边界条件（起点、终点、最大速度/加速度/加加速度）
- **`vp::TrapezoidalPlanner`** - 梯形速度规划器
- **`vp::MultiVelocityPlanner`** - 多自由度规划器（工厂模式）

### 主要方法

```cpp
// 规划轨迹
auto traj = planner.planTrajs(isNormalized);

// 获取运动学状态
auto states = planner.planKStates(isNormalized);

// 查询特定时间的状态
auto state = planner.getKState(time);

// 获取终点位置
auto end_pos = planner.getEndTraj();
```

## 下一步

- 📖 阅读完整的 [README.md](README.md)
- 💻 查看 [examples/](examples/) 目录的更多示例
- 🔧 查看 [EXTRACTION_SUMMARY.md](EXTRACTION_SUMMARY.md) 了解模块详情
- 🐛 遇到问题？提交 Issue！

## 支持的平台

- ✅ Linux (Ubuntu 20.04+)
- ✅ Windows (MSVC 2019+)
- ✅ macOS (Clang 10+)

## 许可证

Apache License 2.0 - 可自由用于商业和开源项目
