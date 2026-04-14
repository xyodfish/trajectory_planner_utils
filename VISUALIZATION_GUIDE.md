# Visualization Guide 📊

本指南介绍如何使用 `example_visualization` 生成可用于调参、汇报和论文插图的轨迹图。

核心实现基于 [matplotlib-cpp](https://github.com/lava/matplotlib-cpp)（C++ 调用 Python matplotlib）。

## 1. 你会得到什么图

运行示例后会生成 4 张 PNG：

1. `velocity_comparison.png`
对比 TVP 与 DSVP 的速度曲线。

2. `double_s_kinematics.png`
DSVP 的位移/速度/加速度/jerk 四联图。

3. `cartesian_trajectory.png`
笛卡尔路径（X-Y 投影）+ XYZ 随时间变化。

4. `acceleration_comparison.png`
TVP 与 DSVP 加速度曲线对比。

## 2. 环境依赖

### Ubuntu / Debian

```bash
sudo apt install python3-dev python3-numpy python3-matplotlib
```

校验：

```bash
python3 -c "import matplotlib, numpy; print('Visualization dependencies OK')"
```

## 3. 构建与运行

在仓库根目录执行：

```bash
./build.sh
```

CMake 识别成功时会看到：

```text
Visualization example enabled (matplotlib-cpp)
```

然后运行：

```bash
cd build/examples
./example_visualization
```

## 4. 结果检查清单

- 程序是否输出 `All plots saved successfully!`
- 当前目录下是否出现 4 张 PNG
- 图片尺寸是否满足你的使用场景（报告/论文/网页）

## 5. 最小绘图片段

```cpp
#include "third_party/matplotlibcpp.h"
namespace plt = matplotlibcpp;

std::vector<double> x = {0, 1, 2, 3, 4};
std::vector<double> y = {0, 1, 4, 9, 16};

plt::figure_size(1200, 800);
plt::plot(x, y, {{"color", "blue"}, {"linewidth", "2"}});
plt::title("Simple Plot");
plt::xlabel("X");
plt::ylabel("Y");
plt::grid(true);
plt::save("simple_plot.png");
plt::close();  // 强烈建议：避免退出时异常
```

## 6. 轨迹数据提取模板

### TVP

```cpp
vp::TrapezoidalPlanner planner({bc}, "TVP");
auto traj = planner.planTrajs();

std::vector<double> t, pos, vel, acc;
for (const auto& p : traj) {
    t.push_back(p[0]);
    pos.push_back(p[1]);
    vel.push_back(p[2]);
    acc.push_back(p[3]);
}
```

### DSVP

```cpp
vp::DoubleSPlanner planner({bc}, "DSVP", 0.95);
auto traj = planner.planTrajs();

std::vector<double> t, pos, vel, acc, jerk;
for (const auto& p : traj) {
    t.push_back(p[0]);
    pos.push_back(p[1]);
    vel.push_back(p[2]);
    acc.push_back(p[3]);
    jerk.push_back(p[4]);
}
```

### Cartesian

```cpp
vp::StraightTrajectory straight(start_pose, goal_pose, {bc}, "DSVP");
auto cart = straight.getTrajs();

std::vector<double> t, x, y, z;
for (const auto& p : cart) {
    t.push_back(p[0]);
    x.push_back(p[1]);
    y.push_back(p[2]);
    z.push_back(p[3]);
}
```

## 7. 实用绘图建议

- 图对比场景：固定坐标轴范围，减少视觉误导
- 参数调优场景：同时画 `vel/acc/jerk`，定位不平滑根因
- 报告场景：统一配色与字号，图更专业
- 批量生成：每张图结束后调用 `plt::close()`

## 8. 常见问题（排障版）

### Q1: CMake 提示 Python/NumPy 未找到

先安装依赖，再清理构建目录重建：

```bash
sudo apt install python3-dev python3-numpy python3-matplotlib
rm -rf build
./build.sh
```

### Q2: 程序结束时崩溃或异常退出

优先检查是否在流程末尾调用了：

```cpp
plt::close();
```

### Q3: 图片没生成

按顺序检查：

1. `example_visualization` 是否真正被编译出来
2. 当前目录是否有写权限
3. 程序是否在异常中提前退出

### Q4: 图片太糊

提高分辨率：

```cpp
plt::figure_size(2400, 1600);
```

## 9. 推荐工作流

1. 先用默认参数跑出 4 张基础图。
2. 改动 `max_vel/max_acc/max_jerk/delta_t`，重复生成并对比。
3. 将对比图纳入参数评审或控制策略讨论。

一句话总结：先看图，再调参；让数据说话，让机器人少“抖”一点 🤖

## 10. 相关文档

- [README.md](README.md)
- [QUICKSTART.md](QUICKSTART.md)
- [QUICK_REFERENCE.md](QUICK_REFERENCE.md)
- [PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md)
