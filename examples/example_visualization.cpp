/**
 * @file example_visualization.cpp
 * @brief Visualization examples using matplotlib-cpp
 * 
 * This example demonstrates how to visualize velocity profiles,
 * acceleration curves, and trajectory comparisons.
 */

#include <iostream>
#include <vector>
#include <string>
#include "vp/trapezoidal_planner.h"
#include "vp/double_s_planner.h"
#include "vp/geometry_trajectory/straight_trajectory.h"
#include "third_party/matplotlibcpp.h"

namespace plt = matplotlibcpp;

/**
 * @brief Plot velocity profile comparison
 */
void plotVelocityComparison(const std::vector<std::vector<double>>& tvp_traj,
                           const std::vector<std::vector<double>>& dsvp_traj) {
    std::cout << "\n=== Plotting Velocity Profile Comparison ===" << std::endl;
    
    // Extract time and velocity data
    std::vector<double> tvp_time, tvp_vel, dsvp_time, dsvp_vel;
    
    for (const auto& point : tvp_traj) {
        tvp_time.push_back(point[0]);
        tvp_vel.push_back(point[2]);  // velocity is at index 2
    }
    
    for (const auto& point : dsvp_traj) {
        dsvp_time.push_back(point[0]);
        dsvp_vel.push_back(point[2]);
    }
    
    // Create figure
    plt::figure_size(1200, 800);
    
    // Plot velocity profiles
    plt::plot(tvp_time, tvp_vel, {{"label", "Trapezoidal"}});
    plt::plot(dsvp_time, dsvp_vel, {{"label", "Double-S (S-Curve)"}});
    
    plt::title("Velocity Profile Comparison: Trapezoidal vs Double-S");
    plt::xlabel("Time (s)");
    plt::ylabel("Velocity (m/s)");
    plt::legend();
    plt::grid(true);
    
    plt::save("./velocity_comparison.png");
    std::cout << "Saved: velocity_comparison.png" << std::endl;
}

/**
 * @brief Plot complete kinematic states for Double-S planner
 */
void plotDoubleSKinematics(const std::vector<std::vector<double>>& traj) {
    std::cout << "\n=== Plotting Double-S Kinematic States ===" << std::endl;
    
    std::vector<double> time, pos, vel, acc, jerk;
    
    for (const auto& point : traj) {
        time.push_back(point[0]);
        pos.push_back(point[1]);
        vel.push_back(point[2]);
        acc.push_back(point[3]);
        jerk.push_back(point[4]);
    }
    
    // Create 4 subplots
    plt::figure_size(1400, 1000);
    
    // Position
    plt::subplot(4, 1, 1);
    plt::plot(time, pos, {{"color", "blue"}});
    plt::title("Double-S Planner - Complete Kinematic States");
    plt::ylabel("Position (m)");
    plt::grid(true);
    
    // Velocity
    plt::subplot(4, 1, 2);
    plt::plot(time, vel, {{"color", "green"}});
    plt::ylabel("Velocity (m/s)");
    plt::grid(true);
    
    // Acceleration
    plt::subplot(4, 1, 3);
    plt::plot(time, acc, {{"color", "orange"}});
    plt::ylabel("Accel (m/s²)");
    plt::grid(true);
    
    // Jerk
    plt::subplot(4, 1, 4);
    plt::plot(time, jerk, {{"color", "red"}});
    plt::xlabel("Time (s)");
    plt::ylabel("Jerk (m/s³)");
    plt::grid(true);
    
    plt::save("./double_s_kinematics.png");
    std::cout << "Saved: double_s_kinematics.png" << std::endl;
}

/**
 * @brief Plot Cartesian straight line trajectory
 */
void plotCartesianTrajectory(const std::vector<std::vector<double>>& traj) {
    std::cout << "\n=== Plotting Cartesian Straight Line Trajectory ===" << std::endl;
    
    std::vector<double> time, x, y, z;
    
    for (const auto& point : traj) {
        time.push_back(point[0]);
        x.push_back(point[1]);   // x position
        y.push_back(point[2]);   // y position
        z.push_back(point[3]);   // z position
    }
    
    // Create figure with 2 subplots
    plt::figure_size(1400, 600);
    
    // 3D-like view (x-y projection)
    plt::subplot(1, 2, 1);
    plt::plot(x, y, {{"color", "blue"}, {"linewidth", "2"}});
    plt::title("Cartesian Trajectory (X-Y Projection)");
    plt::xlabel("X (m)");
    plt::ylabel("Y (m)");
    plt::grid(true);
    plt::axis("equal");
    
    // Position over time
    plt::subplot(1, 2, 2);
    plt::plot(time, x, {{"label", "X"}, {"color", "red"}});
    plt::plot(time, y, {{"label", "Y"}, {"color", "green"}});
    plt::plot(time, z, {{"label", "Z"}, {"color", "blue"}});
    plt::title("Position Components Over Time");
    plt::xlabel("Time (s)");
    plt::ylabel("Position (m)");
    plt::legend();
    plt::grid(true);
    
    plt::save("./cartesian_trajectory.png");
    std::cout << "Saved: cartesian_trajectory.png" << std::endl;
}

/**
 * @brief Plot acceleration comparison
 */
void plotAccelerationComparison(const std::vector<std::vector<double>>& tvp_traj,
                               const std::vector<std::vector<double>>& dsvp_traj) {
    std::cout << "\n=== Plotting Acceleration Comparison ===" << std::endl;
    
    std::vector<double> tvp_time, tvp_acc, dsvp_time, dsvp_acc;
    
    for (const auto& point : tvp_traj) {
        tvp_time.push_back(point[0]);
        tvp_acc.push_back(point[3]);  // acceleration at index 3
    }
    
    for (const auto& point : dsvp_traj) {
        dsvp_time.push_back(point[0]);
        dsvp_acc.push_back(point[3]);
    }
    
    plt::figure_size(1200, 600);
    
    plt::plot(tvp_time, tvp_acc, {{"label", "Trapezoidal"}, {"color", "red"}});
    plt::plot(dsvp_time, dsvp_acc, {{"label", "Double-S"}, {"color", "blue"}});
    
    plt::title("Acceleration Profile Comparison");
    plt::xlabel("Time (s)");
    plt::ylabel("Acceleration (m/s²)");
    plt::legend();
    plt::grid(true);
    
    // Add horizontal line at zero
    std::vector<double> zero_line(tvp_time.size(), 0.0);
    plt::plot(tvp_time, zero_line, {{"color", "black"}, {"linestyle", "--"}, {"linewidth", "1"}});
    
    plt::save("./acceleration_comparison.png");
    std::cout << "Saved: acceleration_comparison.png" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Velocity Planning Visualization Demo" << std::endl;
    std::cout << "========================================" << std::endl;
    
    try {
        // Define boundary conditions
        vp::BCs<double> bc;
        bc.start_state.pos = 0.0;
        bc.start_state.vel = 0.0;
        bc.goal_state.pos  = 1.0;
        bc.goal_state.vel  = 0.0;
        bc.max_vel         = 0.5;
        bc.max_acc         = 0.3;
        bc.max_jerk        = 0.5;
        bc.delta_t         = 0.01;
        
        // Generate Trapezoidal trajectory
        std::cout << "\nGenerating Trapezoidal trajectory..." << std::endl;
        vp::TrapezoidalPlanner tvp_planner({bc}, "TVP");
        auto tvp_traj = tvp_planner.planTrajs();
        std::cout << "TVP: " << tvp_traj.size() << " points" << std::endl;
        
        // Generate Double-S trajectory
        std::cout << "Generating Double-S trajectory..." << std::endl;
        vp::DoubleSPlanner dsvp_planner({bc}, "DSVP", 0.95);
        auto dsvp_traj = dsvp_planner.planTrajs();
        std::cout << "DSVP: " << dsvp_traj.size() << " points" << std::endl;
        
        // Generate Cartesian straight line trajectory
        std::cout << "Generating Cartesian straight line trajectory..." << std::endl;
        std::vector<double> start_pose = {0.0, 0.0, 0.5, 0.0, 0.0, 0.0};
        std::vector<double> goal_pose  = {0.5, 0.3, 0.8, 0.0, 0.0, 1.57};
        vp::StraightTrajectory straight_traj(start_pose, goal_pose, {bc}, "DSVP");
        auto cartesian_traj = straight_traj.getTrajs();
        std::cout << "Cartesian: " << cartesian_traj.size() << " points" << std::endl;
        
        // Create visualizations
        std::cout << "\nCreating visualizations..." << std::endl;
        
        // 1. Velocity comparison
        plotVelocityComparison(tvp_traj, dsvp_traj);
        
        // 2. Double-S complete kinematics
        plotDoubleSKinematics(dsvp_traj);
        
        // 3. Cartesian trajectory
        plotCartesianTrajectory(cartesian_traj);
        
        // 4. Acceleration comparison
        plotAccelerationComparison(tvp_traj, dsvp_traj);
        
        std::cout << "\n========================================" << std::endl;
        std::cout << "All plots saved successfully!" << std::endl;
        std::cout << "Generated files:" << std::endl;
        std::cout << "  - velocity_comparison.png" << std::endl;
        std::cout << "  - double_s_kinematics.png" << std::endl;
        std::cout << "  - cartesian_trajectory.png" << std::endl;
        std::cout << "  - acceleration_comparison.png" << std::endl;
        std::cout << "========================================" << std::endl;
        
        // Note: plt::show() may cause segmentation fault on exit
        // Use plt::save() instead, which works reliably
        // plt::show();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    // Explicitly close all figures to avoid segfault
    plt::close();
    
    return 0;
}
