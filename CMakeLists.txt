cmake_minimum_required(VERSION 3.16.0)
project(velocity_planning VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options
option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_TESTS "Build unit tests" OFF)
option(BUILD_UNIFIED_TRAJECTORY_MODULES "Build unified trajectory planning modules into velocity_planning" ON)
option(BUILD_UNIFIED_CL_MODULES "Build circle-line trajectory modules (requires yaml-cpp)" ON)
option(BUILD_UNIFIED_SAS_MODULES "Build SAS trajectory modules" ON)
option(BUILD_UNIFIED_TOTG_ADAPTER "Build TOTG path adapter (requires yaml-cpp + totg_Trajectory.h)" OFF)
option(BUILD_UNIFIED_PLOTTING "Enable plotting helpers that depend on matplotlib-cpp/Python" OFF)

set(VP_CL_MODULES_COMPILED OFF)
set(VP_SAS_MODULES_COMPILED OFF)
set(VP_TOTG_ADAPTER_COMPILED OFF)

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

if(BUILD_UNIFIED_TRAJECTORY_MODULES)
    set(VP_CL_MODULES_COMPILED OFF)
    set(VP_SAS_MODULES_COMPILED OFF)
    set(VP_TOTG_ADAPTER_COMPILED OFF)

    set(VP_TP_CORE_SOURCES
        src/trajectory_plan/velocity_planning/VelocityPlannerCompat.cpp
        src/trajectory_plan/velocity_planning/trapezoidal_velocity_trajectory/TrapezoidalVelocityTrajectory.cpp
        src/trajectory_plan/geometry_trajectory/circle_trajectory/CircleTrajectory.cpp
        src/trajectory_plan/geometry_trajectory/ellipse_trajectory/EllipseTrajectory.cpp
        src/trajectory_plan/geometry_trajectory/straight_arc_transition/StraightArcTransition.cpp
        src/trajectory_plan/geometry_trajectory/straight_trajectory/straightTrajectory.cpp
        src/trajectory_plan/geometry_trajectory/straight_trajectory/straightTrajectoryAgileX.cpp
        src/trajectory_plan/traj_io_std.cpp
        src/trajectory_plan/traj_arc_utils.cpp
    )
    target_sources(velocity_planning PRIVATE ${VP_TP_CORE_SOURCES})

    set(VP_TP_SHORT_INCLUDE_SUBDIRS
        common
        trajectory_plan
        trajectory_plan/velocity_planning
        trajectory_plan/velocity_planning/trapezoidal_velocity_trajectory
        trajectory_plan/geometry_trajectory/circle_trajectory
        trajectory_plan/geometry_trajectory/ellipse_trajectory
        trajectory_plan/geometry_trajectory/straight_arc_transition
        trajectory_plan/geometry_trajectory/straight_trajectory
        trajectory_plan/geometry_trajectory/sas_trajectory
        trajectory_plan/cl
    )

    foreach(subdir IN LISTS VP_TP_SHORT_INCLUDE_SUBDIRS)
        target_include_directories(velocity_planning
            PUBLIC
                $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/vp/${subdir}>
                $<INSTALL_INTERFACE:include/vp/${subdir}>
        )
    endforeach()

    target_compile_definitions(velocity_planning PRIVATE DBG_MACRO_NO_WARNING)

    if(BUILD_UNIFIED_TOTG_ADAPTER)
        find_package(yaml-cpp QUIET)
        find_path(VP_TP_TOTG_INCLUDE_DIR totg_Trajectory.h)
        if(yaml-cpp_FOUND AND VP_TP_TOTG_INCLUDE_DIR)
            target_sources(velocity_planning PRIVATE src/trajectory_plan/traj.cpp)
            target_include_directories(velocity_planning PRIVATE ${VP_TP_TOTG_INCLUDE_DIR})
            target_link_libraries(velocity_planning PUBLIC yaml-cpp)
            target_compile_definitions(velocity_planning PRIVATE VP_TP_ENABLE_TOTG_ADAPTER)
            set(VP_TOTG_ADAPTER_COMPILED ON)
        else()
            message(WARNING "Unified TOTG adapter disabled: yaml-cpp or totg_Trajectory.h not found")
            target_sources(velocity_planning PRIVATE src/trajectory_plan/traj_basic.cpp)
        endif()
    else()
        target_sources(velocity_planning PRIVATE src/trajectory_plan/traj_basic.cpp)
    endif()

    if(BUILD_UNIFIED_CL_MODULES)
        find_package(yaml-cpp QUIET)
        if(yaml-cpp_FOUND)
            target_sources(velocity_planning PRIVATE
                src/trajectory_plan/traj_circle_line.cpp
                src/trajectory_plan/cl/circleLineTrajGeneration.cpp
                src/trajectory_plan/cl/normalizedCircleLineTrajPlanner.cpp
                src/trajectory_plan/cl/splicedCircleLineTrajPlanner.cpp
            )
            target_link_libraries(velocity_planning PUBLIC yaml-cpp)
            target_compile_definitions(velocity_planning PUBLIC VP_UNIFIED_CL_MODULES_ENABLED=1)
            set(VP_CL_MODULES_COMPILED ON)
        else()
            target_compile_definitions(velocity_planning PUBLIC VP_UNIFIED_CL_MODULES_ENABLED=0)
            message(WARNING "yaml-cpp not found, unified CL modules are disabled")
        endif()
    else()
        target_compile_definitions(velocity_planning PUBLIC VP_UNIFIED_CL_MODULES_ENABLED=0)
    endif()

    if(BUILD_UNIFIED_SAS_MODULES)
        target_sources(velocity_planning PRIVATE
            src/trajectory_plan/geometry_trajectory/sas_trajectory/sas_segments.cpp
            src/trajectory_plan/geometry_trajectory/sas_trajectory/sasTrajectory.cpp
            src/trajectory_plan/geometry_trajectory/sas_trajectory/sasTimeUniformTraj.cpp
            src/trajectory_plan/geometry_trajectory/sas_trajectory/sasArcTimeNonUniformTraj.cpp
            src/trajectory_plan/geometry_trajectory/sas_trajectory/multiPointsTrajectory.cpp
        )
        target_compile_definitions(velocity_planning PUBLIC VP_UNIFIED_SAS_MODULES_ENABLED=1)
        set(VP_SAS_MODULES_COMPILED ON)
    else()
        target_compile_definitions(velocity_planning PUBLIC VP_UNIFIED_SAS_MODULES_ENABLED=0)
    endif()

    if(BUILD_UNIFIED_PLOTTING)
        find_package(Python3 COMPONENTS Development NumPy QUIET)
        if(Python3_FOUND)
            target_include_directories(velocity_planning
                PUBLIC
                    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/third_party>
                    $<INSTALL_INTERFACE:include/third_party>
            )
            target_link_libraries(velocity_planning PUBLIC Python3::Python)
            target_compile_definitions(velocity_planning PUBLIC VP_TP_ENABLE_PLOTTING)
        else()
            message(WARNING "Python3 dev headers not found, unified plotting support disabled")
        endif()
    endif()
endif()

# Set library properties
set_target_properties(velocity_planning PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    POSITION_INDEPENDENT_CODE ON
)

# Link libraries
if(EIGEN3_FOUND)
    target_link_libraries(velocity_planning PUBLIC Eigen3::Eigen)
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
message(STATUS "  Unified modules:${BUILD_UNIFIED_TRAJECTORY_MODULES}")
message(STATUS "  Unified CL:     ${BUILD_UNIFIED_CL_MODULES}")
message(STATUS "  Unified SAS:    ${BUILD_UNIFIED_SAS_MODULES}")
message(STATUS "  Unified TOTG:   ${BUILD_UNIFIED_TOTG_ADAPTER}")
message(STATUS "  Unified plot:   ${BUILD_UNIFIED_PLOTTING}")
message(STATUS "  CL compiled:    ${VP_CL_MODULES_COMPILED}")
message(STATUS "  SAS compiled:   ${VP_SAS_MODULES_COMPILED}")
message(STATUS "  TOTG compiled:  ${VP_TOTG_ADAPTER_COMPILED}")
message(STATUS "  Eigen3 found:   ${EIGEN3_FOUND}")
message(STATUS "==============================================")
