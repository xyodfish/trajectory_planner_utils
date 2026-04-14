/**
 * @file example_comparison.cpp
 * @brief Example: Compare Trapezoidal vs Double-S velocity profiles
 */

#include <iostream>
#include <vector>
#include "vp/multi_velocity_planner.h"

int main() {
    std::cout << "=== Velocity Planner Comparison: Trapezoidal vs Double-S ===" << std::endl;

    // Define boundary conditions
    vp::KinematicState<double> start_state;
    start_state.pos = 0.0;
    start_state.vel = 0.0;

    vp::KinematicState<double> goal_state;
    goal_state.pos = 1.0;
    goal_state.vel = 0.0;

    vp::BCs<double> bc;
    bc.start_state = start_state;
    bc.goal_state  = goal_state;
    bc.max_vel     = 0.5;
    bc.max_acc     = 0.3;
    bc.max_jerk    = 0.5;
    bc.delta_t     = 0.01;

    try {
        // Test Trapezoidal
        std::cout << "\n--- Trapezoidal Velocity Profile ---" << std::endl;
        vp::MultiVelocityPlanner tvp_planner({bc}, "TVP");
        auto tvp_traj = tvp_planner.getTrajs(false);
        
        std::cout << "Total time: " << (tvp_traj.empty() ? 0 : tvp_traj.back()[0]) << " s" << std::endl;
        std::cout << "Total points: " << tvp_traj.size() << std::endl;

        // Test Double-S
        std::cout << "\n--- Double-S (S-Curve) Velocity Profile ---" << std::endl;
        vp::MultiVelocityPlanner dsvp_planner({bc}, "DSVP");
        auto dsvp_traj = dsvp_planner.getTrajs(false);
        
        std::cout << "Total time: " << (dsvp_traj.empty() ? 0 : dsvp_traj.back()[0]) << " s" << std::endl;
        std::cout << "Total points: " << dsvp_traj.size() << std::endl;

        // Comparison
        std::cout << "\n--- Comparison Summary ---" << std::endl;
        if (!tvp_traj.empty() && !dsvp_traj.empty()) {
            double tvp_time = tvp_traj.back()[0];
            double dsvp_time = dsvp_traj.back()[0];
            
            std::cout << "Time difference: " << (dsvp_time - tvp_time) << " s" << std::endl;
            std::cout << "Double-S is " << ((dsvp_time > tvp_time) ? "slower" : "faster") 
                     << " than Trapezoidal" << std::endl;
            std::cout << "\nKey differences:" << std::endl;
            std::cout << "- Trapezoidal: Discontinuous acceleration (infinite jerk)" << std::endl;
            std::cout << "- Double-S: Continuous acceleration with limited jerk" << std::endl;
            std::cout << "- Double-S provides smoother motion, less vibration" << std::endl;
            std::cout << "- Trapezoidal is simpler and faster to compute" << std::endl;
        }

    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
