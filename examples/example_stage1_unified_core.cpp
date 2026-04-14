#include <iostream>
#include <vector>

#include "vp/trajectory_plan/geometry_trajectory/straight_trajectory/straightTrajectory.h"
#include "vp/trajectory_plan/velocity_planning/VelocityPlannerCompat.h"

int main() {
    std::vector<vp::tp::BCs<double>> bcs(1);
    vp::tp::initDefaultBCs(bcs, 0.4, 0.3, 0.5, 0.01);
    bcs[0].s_state.pos = 0.0;
    bcs[0].g_state.pos = 1.0;

    vp::tp::VelocityPlannerCompat velocityPlanner(bcs, "DSVP");
    auto rawTraj = velocityPlanner.getTrajs(false);
    if (rawTraj.empty()) {
        std::cerr << "stage1 failed: empty velocity trajectory" << std::endl;
        return 1;
    }

    std::vector<double> startPose{0.0, 0.0, 0.5, 0.0, 0.0, 0.0};
    std::vector<double> goalPose{0.3, 0.2, 0.8, 0.0, 0.0, 1.0};
    vp::tp::StraightTrajectory planner(startPose, goalPose, bcs, "DSVP");
    auto cartTraj = planner.getTrajs();
    if (cartTraj.empty()) {
        std::cerr << "stage1 failed: empty cartesian trajectory" << std::endl;
        return 1;
    }

    std::cout << "[stage1] raw points=" << rawTraj.size() << ", cartesian points=" << cartTraj.size() << std::endl;
    return 0;
}
