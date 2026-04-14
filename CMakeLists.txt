cmake_minimum_required(VERSION 3.16.0)
project(velocity_planning VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options
option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_TESTS "Build unit tests" OFF)

# Find dependencies
find_package(Eigen3 QUIET)

# Include directories - use target-based approach
set(VELOCITY_PLANNING_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

# Source files
set(SOURCES
    # Curve interface implementations
    src/curve_interface/constant_jerk_trajectory.cpp
    src/curve_interface/piecewise_trajectory.cpp
    
    # Velocity planners
    src/trapezoidal/trapezoidal_planner.cpp
    src/double_s/double_s_planner.cpp
    src/multi_velocity_planner.cpp
    
    # Geometry trajectory
    src/geometry_trajectory/straight_trajectory.cpp
)

# Create shared library
add_library(velocity_planning SHARED ${SOURCES})

# Set include directories for the library
target_include_directories(velocity_planning
    PUBLIC
        $<BUILD_INTERFACE:${VELOCITY_PLANNING_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${EIGEN3_INCLUDE_DIR}
)

# Set library properties
set_target_properties(velocity_planning PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    POSITION_INDEPENDENT_CODE ON
)

# Link libraries
if(EIGEN3_FOUND)
    target_link_libraries(velocity_planning Eigen3::Eigen)
endif()

# Install targets
install(TARGETS velocity_planning
    EXPORT velocity_planning-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/vp
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
)

# Build examples
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Build tests
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Print configuration summary
message(STATUS "==============================================")
message(STATUS "Velocity Planning Library Configuration:")
message(STATUS "  Version:        ${PROJECT_VERSION}")
message(STATUS "  Build type:     ${CMAKE_BUILD_TYPE}")
message(STATUS "  Examples:       ${BUILD_EXAMPLES}")
message(STATUS "  Tests:          ${BUILD_TESTS}")
message(STATUS "  Eigen3 found:   ${EIGEN3_FOUND}")
message(STATUS "==============================================")
