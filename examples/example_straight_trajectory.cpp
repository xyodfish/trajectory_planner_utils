/**
 * @file example_straight_trajectory.cpp
 * @brief Example: Cartesian space straight line trajectory planning
 */

#include <iostream>
#include <vector>
#include "vp/geometry_trajectory/straight_trajectory.h"

int main() {
    std::cout << "=== Cartesian Space Straight Line Trajectory Example ===" << std::endl;

    // Define start and goal poses [x, y, z, rx, ry, rz]
    // Using Euler angles for orientation (in radians)
    std::vector<double> start_pose = {0.0, 0.0, 0.5, 0.0, 0.0, 0.0};  // Start position
    std::vector<double> goal_pose  = {0.5, 0.3, 0.8, 0.0, 0.0, 1.57}; // Goal position with rotation

    std::cout << "\nStart pose: [";
    for (size_t i = 0; i < start_pose.size(); ++i) {
        std::cout << start_pose[i];
        if (i < start_pose.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    std::cout << "Goal pose:  [";
    for (size_t i = 0; i < goal_pose.size(); ++i) {
        std::cout << goal_pose[i];
        if (i < goal_pose.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // Define boundary conditions for normalized time planning
    vp::KinematicState<double> start_state;
    start_state.pos = 0.0;
    start_state.vel = 0.0;

    vp::KinematicState<double> goal_state;
    goal_state.pos = 1.0;  // Normalized distance
    goal_state.vel = 0.0;

    vp::BCs<double> bc;
    bc.start_state = start_state;
    bc.goal_state  = goal_state;
    bc.max_vel     = 0.5;   // m/s (normalized velocity)
    bc.max_acc     = 0.3;   // m/s²
    bc.max_jerk    = 0.5;   // m/s³ (for S-curve)
    bc.delta_t     = 0.01;  // 10ms time step

    try {
        std::cout << "\n--- Testing with Trapezoidal Velocity Profile ---" << std::endl;
        
        // Create straight trajectory planner with TVP
        vp::StraightTrajectory straight_traj_tvp(start_pose, goal_pose, {bc}, "TVP");
        
        auto traj_tvp = straight_traj_tvp.getTrajs();
        
        std::cout << "Total trajectory points: " << traj_tvp.size() << std::endl;
        if (!traj_tvp.empty()) {
            std::cout << "Total time: " << traj_tvp.back()[0] << " s" << std::endl;
            
            // Print first few points
            std::cout << "\nFirst 3 trajectory points:" << std::endl;
            std::cout << "Time(s) | X(m)    | Y(m)    | Z(m)    | RX(rad) | RY(rad) | RZ(rad)" << std::endl;
            std::cout << "--------|---------|---------|---------|---------|---------|--------" << std::endl;
            
            size_t print_count = std::min(traj_tvp.size(), static_cast<size_t>(3));
            for (size_t i = 0; i < print_count; ++i) {
                const auto& point = traj_tvp[i];
                std::cout << point[0] << "      | " 
                         << point[1] << " | " << point[2] << " | " << point[3] << " | "
                         << point[4] << " | " << point[5] << " | " << point[6] << std::endl;
            }
        }

        std::cout << "\n--- Testing with Double-S (S-Curve) Velocity Profile ---" << std::endl;
        
        // Create straight trajectory planner with DSVP
        vp::StraightTrajectory straight_traj_dsvp(start_pose, goal_pose, {bc}, "DSVP");
        
        auto traj_dsvp = straight_traj_dsvp.getTrajs();
        
        std::cout << "Total trajectory points: " << traj_dsvp.size() << std::endl;
        if (!traj_dsvp.empty()) {
            std::cout << "Total time: " << traj_dsvp.back()[0] << " s" << std::endl;
            
            // Show velocity information
            if (traj_dsvp.size() > 1) {
                double dt = traj_dsvp[1][0] - traj_dsvp[0][0];
                if (dt > 1e-6) {
                    double vx = (traj_dsvp[1][1] - traj_dsvp[0][1]) / dt;
                    double vy = (traj_dsvp[1][2] - traj_dsvp[0][2]) / dt;
                    double vz = (traj_dsvp[1][3] - traj_dsvp[0][3]) / dt;
                    double linear_vel = std::sqrt(vx*vx + vy*vy + vz*vz);
                    std::cout << "Initial linear velocity: " << linear_vel << " m/s" << std::endl;
                }
            }
        }

        std::cout << "\nNote: Straight trajectory provides Cartesian space linear motion" << std::endl;
        std::cout << "with smooth velocity profiles for precise path following." << std::endl;

    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
