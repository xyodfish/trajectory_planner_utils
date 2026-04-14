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

# Example: Visualization with matplotlib-cpp
find_package(Python3 COMPONENTS Development NumPy)
if(Python3_FOUND AND Python3_NumPy_FOUND)
    add_executable(example_visualization example_visualization.cpp)
    target_link_libraries(example_visualization velocity_planning)
    target_include_directories(example_visualization PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_link_libraries(example_visualization Python3::Python Python3::NumPy)
    
    # Find Python libraries
    execute_process(COMMAND python3-config --ldflags OUTPUT_VARIABLE PYTHON_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
    target_link_options(example_visualization PRIVATE ${PYTHON_LDFLAGS})
    
    message(STATUS "Visualization example enabled (matplotlib-cpp)")
else()
    message(WARNING "Python3 or NumPy not found. Visualization example will be skipped.")
    message(WARNING "To enable visualization, install: sudo apt install python3-dev python3-numpy")
endif()
