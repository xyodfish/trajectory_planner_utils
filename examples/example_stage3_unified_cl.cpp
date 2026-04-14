#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "vp/trajectory_plan/cl/circleLineTrajGeneration.h"
#include "vp/trajectory_plan/cl/normalizedCircleLineTrajPlanner.h"
#include "vp/trajectory_plan/cl/splicedCircleLineTrajPlanner.h"

namespace stage3Example {

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

double maxNorm3(const Trajectory& traj, size_t colX, size_t colY, size_t colZ) {
    double value = 0.0;
    for (const auto& row : traj) {
        const double norm = std::sqrt(row[colX] * row[colX] + row[colY] * row[colY] + row[colZ] * row[colZ]);
        value = std::max(value, norm);
    }
    return value;
}

double xyzGap(const std::vector<double>& tp, const std::vector<double>& pose) {
    const double dx = tp[1] - pose[0];
    const double dy = tp[2] - pose[1];
    const double dz = tp[3] - pose[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double xyzGapTp(const std::vector<double>& lhs, const std::vector<double>& rhs) {
    const double dx = lhs[1] - rhs[1];
    const double dy = lhs[2] - rhs[2];
    const double dz = lhs[3] - rhs[3];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void validateClTraj(const Trajectory& traj,
                    const std::vector<std::vector<double>>& segments,
                    const std::string& name,
                    double speedBound) {
    expectMinColumns(traj, 13, name + " trajectory");
    expectMonotonicTime(traj, name + " trajectory");

    expect(segments.size() >= 2, name + " segment list must contain start and end");

    const double startGap = xyzGap(traj.front(), segments.front());
    const double endGap = xyzGap(traj.back(), segments.back());
    expect(startGap < 1e-8, name + " start pose mismatch");
    expect(endGap < 5e-3, name + " end pose mismatch");

    const double maxSpeed = maxNorm3(traj, 7, 8, 9);
    expect(maxSpeed > 1e-6, name + " speed should be positive");
    expect(maxSpeed <= speedBound * 1.25, name + " speed exceeds expected limit");
}

void validateNormalizedRaw(const Trajectory& raw, const std::string& name) {
    expectMinColumns(raw, 5, name);
    expectMonotonicTime(raw, name);

    double prevLambda = raw.front()[1];
    for (size_t i = 0; i < raw.size(); ++i) {
        const double lambda = raw[i][1];
        expect(lambda >= -1e-8 && lambda <= 1.00001,
               name + " normalized position must stay in [0, 1]");
        expect(lambda + 1e-10 >= prevLambda,
               name + " normalized position must be monotonic");
        prevLambda = lambda;
    }
}

}  // namespace stage3Example

int main() {
    using namespace stage3Example;

    try {
        const std::string sourceDir = std::string(VP_PROJECT_SOURCE_DIR);
        const std::string segmentFile = sourceDir + "/examples/data/cl_segments.yaml";
        const std::string splicedConfigFile = sourceDir + "/examples/data/cl_spliced_config.yaml";
        const std::string generationConfigFile = sourceDir + "/examples/data/cl_generation_spliced.yaml";

        vp::tp::NormalizedCircleLineTrajPlanner normalizedPlanner(segmentFile, "DSVP", {0.22, 0.45, 0.70, 0.01});
        const auto normalizedRaw = normalizedPlanner.getRawTraj();
        const auto normalizedTraj = normalizedPlanner.getTrajs();
        const auto normalizedSegments = normalizedPlanner.getPathSegmentsSgPose();

        validateNormalizedRaw(normalizedRaw, "normalized raw trajectory");
        validateClTraj(normalizedTraj, normalizedSegments, "normalized", 0.22);

        vp::tp::SplicedCircleLineTrajPlanner splicedPlanner(splicedConfigFile, "DSVP");
        const auto splicedRaw = splicedPlanner.getRawTraj();
        const auto splicedTraj = splicedPlanner.getTrajs();
        const auto splicedSegments = splicedPlanner.getPathSegmentsSgPose();

        expectMinColumns(splicedRaw, 5, "spliced raw trajectory");
        validateClTraj(splicedTraj, splicedSegments, "spliced", 0.22);

        expect(normalizedSegments.size() == splicedSegments.size(), "normalized/spliced segment size mismatch");
        expect(xyzGapTp(normalizedTraj.back(), splicedTraj.back()) < 5e-3,
               "normalized/spliced end pose mismatch");

        vp::tp::CircleLineTrajGeneration clGenerator(generationConfigFile);
        expect(!clGenerator.getWpName().empty(), "CircleLineTrajGeneration workpiece name must not be empty");

        clGenerator.setWpPt({0.20, -0.10, 0.30, 0.0, 0.0, 0.0});
        clGenerator.setTrajSpPose({0.01, -0.02, 0.00, 0.0, 0.0, 0.05}, true);

        const auto trajOnWp = clGenerator.getCLTrajOnWp();
        const auto trajOnBase = clGenerator.getCLTrajOnBase();
        const auto trajFromData = clGenerator.getTrajFromData();
        const auto generatorSegments = clGenerator.getSegmentsSgPose();

        validateClTraj(trajOnWp, generatorSegments, "generator_wp", 0.22);
        validateClTraj(trajOnBase, generatorSegments, "generator_base", 0.22);
        expectMinColumns(trajFromData, 13, "generator data trajectory");

        expect(trajOnWp.size() == trajOnBase.size(), "generator wp/base size mismatch");
        expect(trajOnWp.size() == trajFromData.size(), "generator source/runtime size mismatch");

        const double dx = trajOnBase.front()[1] - trajOnWp.front()[1];
        const double dy = trajOnBase.front()[2] - trajOnWp.front()[2];
        const double dz = trajOnBase.front()[3] - trajOnWp.front()[3];
        expect(std::abs(dx - 0.20) < 1e-6, "generator base/wp X offset mismatch");
        expect(std::abs(dy + 0.10) < 1e-6, "generator base/wp Y offset mismatch");
        expect(std::abs(dz - 0.30) < 1e-6, "generator base/wp Z offset mismatch");

        std::cout << "[stage3] normalized(raw/traj)=" << normalizedRaw.size() << "/" << normalizedTraj.size()
                  << ", spliced(raw/traj)=" << splicedRaw.size() << "/" << splicedTraj.size()
                  << ", generator(wp/base/data)=" << trajOnWp.size() << "/" << trajOnBase.size() << "/"
                  << trajFromData.size() << ", segment_points=" << normalizedSegments.size() << std::endl;

        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "stage3 failed: " << ex.what() << std::endl;
        return 1;
    }
}
