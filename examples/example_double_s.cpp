/**
 * @file example_double_s.cpp
 * @brief Example: Double-S (S-Curve) velocity profile planning
 */

#include <iostream>
#include <vector>
#include "vp/double_s_planner.h"

int main() {
    std::cout << "=== Double-S (S-Curve) Velocity Planner Example ===" << std::endl;

    // Define boundary conditions
    vp::KinematicState<double> start_state;
    start_state.pos = 0.0;
    start_state.vel = 0.0;
    start_state.acc = 0.0;

    vp::KinematicState<double> goal_state;
    goal_state.pos = 1.0;  // 1 meter
    goal_state.vel = 0.0;
    goal_state.acc = 0.0;

    vp::BCs<double> bc;
    bc.start_state = start_state;
    bc.goal_state  = goal_state;
    bc.max_vel     = 0.5;   // m/s
    bc.max_acc     = 0.3;   // m/s^2
    bc.max_jerk    = 0.5;   // m/s^3 (jerk limit for smooth motion)
    bc.delta_t     = 0.01;  // 10ms time step

    try {
        // Create Double-S planner with gamma=0.95 (time scaling factor)
        vp::DoubleSPlanner planner({bc}, "DSVP", 0.95);

        std::cout << "\nPlanning S-curve trajectory..." << std::endl;
        std::cout << "Max velocity: " << bc.max_vel << " m/s" << std::endl;
        std::cout << "Max acceleration: " << bc.max_acc << " m/s²" << std::endl;
        std::cout << "Max jerk: " << bc.max_jerk << " m/s³" << std::endl;

        // Plan trajectory
        auto trajectory = planner.planTrajs(false);

        std::cout << "\nTrajectory planned successfully!" << std::endl;
        std::cout << "Total points: " << trajectory.size() << std::endl;
        std::cout << "Total time: " << planner.tTotal << " s" << std::endl;

        // Print first few points
        std::cout << "\nFirst 5 trajectory points:" << std::endl;
        std::cout << "Time(s) | Position(m) | Velocity(m/s) | Acc(m/s²) | Jerk(m/s³)" << std::endl;
        std::cout << "--------|-------------|---------------|-----------|----------" << std::endl;

        size_t print_count = std::min(trajectory.size(), static_cast<size_t>(5));
        for (size_t i = 0; i < print_count; ++i) {
            const auto& point = trajectory[i];
            std::cout << point[0] << "      | " << point[1] << "       | " << point[2] 
                     << "         | " << point[3] << "     | " << point[4] << std::endl;
        }

        // Print last point
        if (!trajectory.empty()) {
            const auto& last = trajectory.back();
            std::cout << "... (last point)" << std::endl;
            std::cout << last[0] << "      | " << last[1] << "       | " << last[2] 
                     << "         | " << last[3] << "     | " << last[4] << std::endl;
        }

        std::cout << "\nNote: S-curve provides smoother motion than trapezoidal profile" << std::endl;
        std::cout << "by limiting jerk (rate of change of acceleration)." << std::endl;

    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
