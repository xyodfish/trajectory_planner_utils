#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "vp/trajectory_plan/geometry_trajectory/sas_trajectory/multiPointsTrajectory.h"

namespace stage2Example {

using Trajectory = std::vector<std::vector<double>>;

void expect(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void expectMinColumns(const Trajectory& traj, size_t minColumns, const std::string& name) {
    expect(!traj.empty(), name + " is empty");
    for (size_t i = 0; i < traj.size(); ++i) {
        expect(traj[i].size() >= minColumns,
               name + " row " + std::to_string(i) + " has insufficient columns");
    }
}

void expectMonotonicTime(const Trajectory& traj, const std::string& name) {
    for (size_t i = 1; i < traj.size(); ++i) {
        expect(traj[i][0] + 1e-12 >= traj[i - 1][0],
               name + " time is not monotonic at row " + std::to_string(i));
    }
}

double maxAbsColumn(const Trajectory& traj, size_t col) {
    double value = 0.0;
    for (const auto& row : traj) {
        value = std::max(value, std::abs(row[col]));
    }
    return value;
}

double maxNorm3(const Trajectory& traj, size_t colX, size_t colY, size_t colZ) {
    double value = 0.0;
    for (const auto& row : traj) {
        const double norm = std::sqrt(row[colX] * row[colX] + row[colY] * row[colY] + row[colZ] * row[colZ]);
        value = std::max(value, norm);
    }
    return value;
}

double xyzDistance(const std::vector<double>& pose, const std::vector<double>& point) {
    const double dx = pose[1] - point[0];
    const double dy = pose[2] - point[1];
    const double dz = pose[3] - point[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace stage2Example

int main() {
    using namespace stage2Example;

    try {
        std::vector<std::vector<double>> waypoints{
            {0.00, 0.00, 0.40, 0.0, 0.0, 0.00},
            {0.08, 0.06, 0.44, 0.0, 0.0, 0.08},
            {0.18, 0.12, 0.50, 0.0, 0.0, 0.22},
            {0.28, 0.20, 0.57, 0.0, 0.0, 0.40},
            {0.36, 0.26, 0.63, 0.0, 0.0, 0.56}};

        vp::tp::MultiPointsTrajectory planner(waypoints, 0.15);

        std::vector<vp::tp::BCs<double>> bcs(1);
        vp::tp::initDefaultBCs(bcs, 0.35, 0.25, 0.45, 0.01);
        bcs[0].s_state.pos = 0.0;

        planner.setPlanner(bcs, "DSVP");

        auto rawTraj = planner.getRawTrajs();
        auto traj = planner.getTrajs();
        auto varySpeedTraj = planner.getVarySpeedTraj();

        expectMinColumns(rawTraj, 5, "raw trajectory");
        expectMinColumns(traj, 19, "uniform trajectory");
        expectMinColumns(varySpeedTraj, 19, "vary-speed trajectory");

        expectMonotonicTime(rawTraj, "raw trajectory");
        expectMonotonicTime(traj, "uniform trajectory");
        expectMonotonicTime(varySpeedTraj, "vary-speed trajectory");

        const double segLength = planner.getSegLength();
        expect(segLength > 1e-8, "segment length must be positive");

        const auto blendRadius = planner.getBlendRadius();
        expect(blendRadius.size() == waypoints.size() - 2, "blend radius size mismatch");
        for (size_t i = 0; i < blendRadius.size(); ++i) {
            expect(blendRadius[i] > 0.0, "blend radius must be positive at index " + std::to_string(i));
        }

        auto plannerBcs = planner.getBoundaryConditions();
        expect(plannerBcs.size() == 1, "planner boundary condition size mismatch");
        expect(std::abs(plannerBcs[0].g_state.pos - segLength) < 1e-8, "goal position is not aligned with segment length");
        expect(planner.getAlgoName() == "DSVP", "planner algorithm mismatch");

        const double startGap = xyzDistance(traj.front(), waypoints.front());
        const double endGap = xyzDistance(traj.back(), waypoints.back());
        const double varyEndGap = xyzDistance(varySpeedTraj.back(), waypoints.back());
        expect(startGap < 1e-8, "uniform trajectory start point mismatch");
        expect(endGap < 5e-3, "uniform trajectory end point mismatch");
        expect(varyEndGap < 5e-3, "vary-speed trajectory end point mismatch");

        const double rawMaxVel = maxAbsColumn(rawTraj, 2);
        const double rawMaxAcc = maxAbsColumn(rawTraj, 3);
        expect(rawMaxVel <= bcs[0].max_vel * 1.01, "raw velocity exceeds configured max velocity");
        expect(rawMaxAcc <= bcs[0].max_acc * 1.05, "raw acceleration exceeds configured max acceleration");

        const double maxUniformCartesianSpeed = maxNorm3(traj, 7, 8, 9);
        const double maxUniformCartesianAcc = maxNorm3(traj, 13, 14, 15);
        const double maxVaryCartesianSpeed = maxNorm3(varySpeedTraj, 7, 8, 9);
        const double maxVaryCartesianAcc = maxNorm3(varySpeedTraj, 13, 14, 15);

        expect(maxUniformCartesianSpeed > 1e-6, "uniform trajectory speed should be positive");
        expect(maxVaryCartesianSpeed > 1e-6, "vary-speed trajectory speed should be positive");

        std::cout << "[stage2] raw=" << rawTraj.size() << ", uniform=" << traj.size()
                  << ", vary=" << varySpeedTraj.size() << ", seg_length=" << segLength
                  << ", blend_count=" << blendRadius.size() << "\n"
                  << "[stage2] max_raw_vel=" << rawMaxVel << ", max_raw_acc=" << rawMaxAcc
                  << ", max_uniform_cart_speed=" << maxUniformCartesianSpeed
                  << ", max_uniform_cart_acc=" << maxUniformCartesianAcc
                  << ", max_vary_cart_speed=" << maxVaryCartesianSpeed
                  << ", max_vary_cart_acc=" << maxVaryCartesianAcc << std::endl;

        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "stage2 failed: " << ex.what() << std::endl;
        return 1;
    }
}
