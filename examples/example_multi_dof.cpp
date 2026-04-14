/**
 * @file example_multi_dof.cpp
 * @brief Example: Multi-DOF velocity planning using factory pattern
 */

#include <iostream>
#include <vector>
#include "vp/multi_velocity_planner.h"

int main() {
    std::cout << "=== Multi-DOF Velocity Planner Example ===" << std::endl;

    // Define 6-DOF boundary conditions (e.g., for a robot arm)
    const int DOF = 6;
    std::vector<vp::BCs<double>> bcs(DOF);

    // Initialize with default values
    vp::MultiVelocityPlanner::initDefaultBCs(bcs, 0.5, 0.3, 0.0, 0.01);

    // Customize goal positions for each DOF
    std::vector<double> goals = {0.5, -0.3, 0.8, 0.2, -0.4, 0.6};
    for (int i = 0; i < DOF; ++i) {
        bcs[i].goal_state.pos = goals[i];
    }

    try {
        // Create multi-DOF planner with trapezoidal profile
        vp::MultiVelocityPlanner planner(bcs, "TVP");

        std::cout << "\nPlanning trajectory for " << DOF << " DOFs..." << std::endl;

        // Get trajectory
        auto trajectory = planner.getTrajs(false);

        std::cout << "Trajectory planned successfully!" << std::endl;
        std::cout << "Total time steps: " << trajectory.size() << std::endl;
        std::cout << "Number of DOFs: " << (trajectory.empty() ? 0 : trajectory[0].size()) << std::endl;

        // Print summary
        if (!trajectory.empty()) {
            std::cout << "\nTrajectory summary:" << std::endl;
            std::cout << "Start time: " << trajectory.front()[0] << " s" << std::endl;
            std::cout << "End time:   " << trajectory.back()[0] << " s" << std::endl;
        }

    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
