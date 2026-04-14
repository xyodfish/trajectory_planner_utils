#include "traj.h"

#include <fstream>
#include <spdlog/spdlog.h>

namespace vp::tp {

bool getTrajL(const std::vector<std::vector<double>>& path,
              const std::string& config_file,
              std::vector<TrajPoint>& traj,
              double time_step) {
    (void)path;
    (void)config_file;
    (void)time_step;
    traj.clear();
    spdlog::error("TOTG adapter is disabled. Rebuild with -DBUILD_UNIFIED_TOTG_ADAPTER=ON");
    return false;
}

void readTrajL(std::vector<TrajPoint>& traj, const std::string& file_name) {
    constexpr unsigned int dim = 6;

    traj.clear();

    std::ifstream input(file_name);
    if (!input.is_open()) {
        spdlog::error("failed to open trajectory file: {}", file_name);
        return;
    }

    Eigen::VectorXd pos(dim);
    Eigen::VectorXd vel(dim);
    TrajPoint point;

    while (input.peek() != EOF) {
        input >> point.time;

        for (unsigned int i = 0; i < dim; ++i) {
            input >> pos[i];
        }

        for (unsigned int i = 0; i < dim; ++i) {
            input >> vel[i];
        }

        point.pos = pos;
        point.vel = vel;
        traj.push_back(point);
    }
}

void writeTrajL(const std::vector<TrajPoint>& traj, std::string file, const std::string& separator) {
    if (traj.empty()) {
        return;
    }

    const unsigned int num = traj.size();
    const unsigned int dim = traj[0].pos.size();

    std::ofstream output(file);
    if (!output.is_open()) {
        spdlog::error("failed to open trajectory file for write: {}", file);
        return;
    }

    for (unsigned int i = 0; i < num; ++i) {
        for (unsigned int j = 0; j < dim; ++j) {
            output << traj[i].pos[j] << separator;
        }

        for (unsigned int j = 0; j < dim; ++j) {
            output << traj[i].vel[j];
            if (j + 1 != dim) {
                output << separator;
            }
        }

        if (i + 1 != num) {
            output << '\n';
        }
    }
}

}  // namespace vp::tp
