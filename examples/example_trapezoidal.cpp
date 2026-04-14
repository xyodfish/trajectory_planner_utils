/**
 * @file example_trapezoidal.cpp
 * @brief Example: Basic trapezoidal velocity profile planning
 */

#include <iostream>
#include <vector>
#include "vp/trapezoidal_planner.h"

int main() {
    std::cout << "=== Trapezoidal Velocity Planner Example ===" << std::endl;

    // Define boundary conditions
    vp::KinematicState<double> start_state;
    start_state.pos = 0.0;
    start_state.vel = 0.0;

    vp::KinematicState<double> goal_state;
    goal_state.pos = 1.0;  // 1 meter
    goal_state.vel = 0.0;

    vp::BCs<double> bc;
    bc.start_state = start_state;
    bc.goal_state  = goal_state;
    bc.max_vel     = 0.5;   // m/s
    bc.max_acc     = 0.3;   // m/s^2
    bc.max_jerk    = 0.0;   // Not used in trapezoidal
    bc.delta_t     = 0.01;  // 10ms time step

    try {
        // Create planner
        vp::TrapezoidalPlanner planner({bc}, "TVP");

        // Plan trajectory
        auto trajectory = planner.planTrajs(false);

        std::cout << "\nTrajectory planned successfully!" << std::endl;
        std::cout << "Total points: " << trajectory.size() << std::endl;

        // Print first few points
        std::cout << "\nFirst 5 trajectory points:" << std::endl;
        std::cout << "Time(s) | Position(m) | Velocity(m/s)" << std::endl;
        std::cout << "--------|-------------|--------------" << std::endl;

        size_t print_count = std::min(trajectory.size(), static_cast<size_t>(5));
        for (size_t i = 0; i < print_count; ++i) {
            const auto& point = trajectory[i];
            std::cout << point[0] << "      | " << point[1] << "       | " << point[2] << std::endl;
        }

        // Print last point
        if (!trajectory.empty()) {
            const auto& last = trajectory.back();
            std::cout << "... (last point)" << std::endl;
            std::cout << last[0] << "      | " << last[1] << "       | " << last[2] << std::endl;
        }

    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
