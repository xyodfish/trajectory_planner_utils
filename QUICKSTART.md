# Quick Start Guide ⚡

5 分钟内把 `trajectory_planner_utils` 跑起来。

## 1. 一条命令构建

```bash
cd trajectory_planner_utils
./build.sh
```

如果你喜欢手动控制参数：

```bash
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=ON -DBUILD_TESTS=OFF
cmake --build . -j$(nproc)
```

## 2. 运行示例程序

```bash
cd build/examples
./example_trapezoidal
./example_double_s
./example_multi_dof
./example_straight_trajectory
./example_comparison
```

## 3. 在你的项目中接入

### CMakeLists.txt（推荐：源码方式集成）

```cmake
add_subdirectory(external/trajectory_planner_utils)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE velocity_planning)
target_include_directories(my_app PRIVATE external/trajectory_planner_utils/include)
```

### `main.cpp` 最小示例

```cpp
#include <iostream>
#include <vp/trapezoidal_planner.h>

int main() {
    vp::BCs<double> bc;
    bc.start_state.pos = 0.0;
    bc.start_state.vel = 0.0;
    bc.goal_state.pos  = 1.0;
    bc.goal_state.vel  = 0.0;
    bc.max_vel         = 0.5;
    bc.max_acc         = 0.3;
    bc.max_jerk        = 0.0;
    bc.delta_t         = 0.01;

    vp::TrapezoidalPlanner planner({bc}, "TVP");
    auto trajectory = planner.planTrajs();

    std::cout << "Generated " << trajectory.size() << " points" << std::endl;
    return 0;
}
```

## 4. 常见参数建议

- `delta_t`: 推荐先用 `0.01`（10ms）
- `TVP`: 原型开发/实时性优先
- `DSVP`: 平滑性优先（务必设置 `max_jerk`）

## 5. 错误处理模板

```cpp
#include <vp/planner_exception.h>

try {
    vp::TrapezoidalPlanner planner({bc}, "TVP");
    auto traj = planner.planTrajs();
} catch (const vp::PlannerException& e) {
    std::cerr << "Error " << e.errorCode() << ": " << e.what() << std::endl;
}
```

常见错误码：
- `1001`: 未提供约束条件
- `1002`: 计算的位置为空
- `1008`: 值超出边界
- `1009`: 输入为空

## 6. 需要画图？

安装可视化依赖后可运行：

```bash
sudo apt install python3-dev python3-numpy python3-matplotlib
cd build/examples
./example_visualization
```

会输出 4 张 PNG 图，适合做调参与报告展示 📊

## 下一步

- 阅读主文档：[README.md](README.md)
- API 速查：[QUICK_REFERENCE.md](QUICK_REFERENCE.md)
- 架构说明：[PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md)
- 可视化细节：[VISUALIZATION_GUIDE.md](VISUALIZATION_GUIDE.md)
