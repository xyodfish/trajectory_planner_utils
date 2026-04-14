cmake_minimum_required(VERSION 3.16.0)

# Example: Basic trapezoidal velocity planning
add_executable(example_trapezoidal example_trapezoidal.cpp)
target_link_libraries(example_trapezoidal velocity_planning)

# Example: Double-S (S-Curve) velocity planning
add_executable(example_double_s example_double_s.cpp)
target_link_libraries(example_double_s velocity_planning)

# Example: Cartesian space straight line trajectory
add_executable(example_straight_trajectory example_straight_trajectory.cpp)
target_link_libraries(example_straight_trajectory velocity_planning)

# Example: Comparison between algorithms
add_executable(example_comparison example_comparison.cpp)
target_link_libraries(example_comparison velocity_planning)

# Example: Multi-DOF planning
add_executable(example_multi_dof example_multi_dof.cpp)
target_link_libraries(example_multi_dof velocity_planning)
