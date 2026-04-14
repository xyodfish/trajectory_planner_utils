# Velocity Planning Library - Quick Reference Card

## 🚀 5分钟快速开始

### 1. 构建
```bash
cd velocity_planning
./build.sh
```

### 2. 运行示例
```bash
cd build/examples
./example_trapezoidal    # 梯形速度规划
./example_double_s       # S-Curve 速度规划  
./example_comparison     # 算法对比
./example_multi_dof      # 多轴规划
```

---

## 📝 常用代码片段

### 梯形速度规划
```cpp
#include <vp/trapezoidal_planner.h>

vp::BCs<double> bc;
bc.start_state.pos = 0.0;
bc.goal_state.pos  = 1.0;
bc.max_vel         = 0.5;
bc.max_acc         = 0.3;
bc.delta_t         = 0.01;

vp::TrapezoidalPlanner planner({bc}, "TVP");
auto traj = planner.planTrajs();
```

### Double-S (S-Curve) 规划
```cpp
#include <vp/double_s_planner.h>

vp::BCs<double> bc;
bc.start_state.pos = 0.0;
bc.goal_state.pos  = 1.0;
bc.max_vel         = 0.5;
bc.max_acc         = 0.3;
bc.max_jerk        = 0.5;  // 重要！
bc.delta_t         = 0.01;

vp::DoubleSPlanner planner({bc}, "DSVP", 0.95);
auto traj = planner.planTrajs();
// traj[i] = [time, pos, vel, acc, jerk]
```

### 多轴规划（工厂模式）
```cpp
#include <vp/multi_velocity_planner.h>

std::vector<vp::BCs<double>> bcs(6);  // 6轴
vp::MultiVelocityPlanner::initDefaultBCs(bcs, 0.5, 0.3, 0.5, 0.01);

// 选择算法
vp::MultiVelocityPlanner planner(bcs, "TVP");   // 或 "DSVP"
auto traj = planner.getTrajs();
```

---

## 🔑 核心API

### 边界条件 (BoundaryConditions)
```cpp
struct BCs {
    KinematicState start_state;  // 起始状态
    KinematicState goal_state;   // 目标状态
    double max_vel;              // 最大速度
    double max_acc;              // 最大加速度
    double max_jerk;             // 最大加加速度
    double delta_t;              // 时间步长
};
```

### 运动学状态 (KinematicState)
```cpp
struct KinematicState {
    double time;   // 时间
    double pos;    // 位置
    double vel;    // 速度
    double acc;    // 加速度
    double jerk;   // 加加速度
};
```

### 规划器接口
```cpp
// 规划轨迹
auto traj = planner.planTrajs(isNormalized);

// 获取运动学状态
auto states = planner.planKStates(isNormalized);

// 查询特定时刻
auto state = planner.getKState(time);

// 获取终点状态
auto end = planner.getEndTraj();
```

---

## ⚙️ 算法选择指南

| 场景 | 推荐算法 | 原因 |
|------|---------|------|
| 快速原型 | Trapezoidal | 简单、快速 |
| 精密装配 | Double-S | 平滑、低振动 |
| 高速运动 | Trapezoidal | 时间最优 |
| 柔性物体 | Double-S | 限制冲击 |
| 教学演示 | 两者对比 | 理解差异 |

---

## 🐛 常见问题

### Q1: 编译错误 "PlannerException not declared"
**A**: 确保包含正确的头文件
```cpp
#include <vp/planner_exception.h>
```

### Q2: Double-S 规划失败
**A**: 检查是否设置了 `max_jerk`
```cpp
bc.max_jerk = 0.5;  // 必须设置！
```

### Q3: 轨迹时间为0
**A**: 检查起点和终点是否相同
```cpp
if (start.pos == goal.pos) {
    // 无需规划
}
```

### Q4: 数值不稳定
**A**: 调整 gamma 参数 (0.8-1.0)
```cpp
vp::DoubleSPlanner planner({bc}, "DSVP", 0.9);
```

---

## 📊 性能参考

| 指标 | Trapezoidal | Double-S |
|------|-------------|----------|
| 计算时间 | ~0.1ms | ~0.5ms |
| 内存占用 | ~1KB | ~5KB |
| 轨迹点数 | 较少 | 较多 |
| 平滑度 | ★★☆☆☆ | ★★★★★ |

*测试环境: Intel i7, 1m位移, dt=0.01s*

---

## 🔗 相关资源

- 📖 完整文档: [README.md](README.md)
- 🚀 快速开始: [QUICKSTART.md](QUICKSTART.md)
- 📋 项目报告: [DELIVERY_REPORT.md](DELIVERY_REPORT.md)
- 💻 示例代码: `examples/` 目录

---

## 📞 获取帮助

1. 查看示例代码
2. 阅读 API 文档
3. 提交 GitHub Issue
4. 查看源代码注释

---

**版本**: v1.0.0  
**许可证**: Apache 2.0  
**最后更新**: 2024-04-14
