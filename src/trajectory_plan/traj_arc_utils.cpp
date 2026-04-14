#include "traj.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace vp::tp {

Eigen::Vector3d calArcCenter(const Eigen::Vector3d& start_pos,
                             const Eigen::Vector3d& end_pos,
                             Eigen::Vector3d axis,
                             double radius,
                             bool smaller_than_pi) {
    Eigen::Vector3d mid = (start_pos + end_pos) / 2;
    double se_2 = (end_pos - start_pos).norm() / 2;
    double a = std::sqrt(std::pow(radius, 2) - std::pow(se_2, 2));

    Eigen::Vector3d se_dir = (end_pos - start_pos) / se_2 / 2;
    axis.normalize();
    Eigen::Vector3d mid_center_dir = axis.cross(se_dir);

    return mid + (a * mid_center_dir) * (smaller_than_pi ? 1 : -1);
}

Eigen::Vector3d calArcAxis(const Eigen::Vector3d& start_pos,
                           const Eigen::Vector3d& end_pos,
                           const Eigen::Vector3d& center_pos,
                           bool positive,
                           const Eigen::Vector3d* mid_pos_ptr) {
    Eigen::Vector3d cs_dir = (start_pos - center_pos).normalized();
    Eigen::Vector3d ce_dir = (end_pos - center_pos).normalized();
    Eigen::Vector3d axis = (cs_dir.cross(ce_dir)).normalized();

    if (axis.norm() == 0) {
        if (mid_pos_ptr == nullptr) {
            throw std::runtime_error("start end center are on one same line, cannot calculate axis");
        }
        Eigen::Vector3d mid_pos = *mid_pos_ptr;
        if (!(mid_pos[0] == end_pos[0] && mid_pos[1] == end_pos[1] && mid_pos[2] == end_pos[2])) {
            Eigen::Vector3d cm_dir = (mid_pos - center_pos).normalized();
            axis = (cs_dir.cross(cm_dir)).normalized();
        }

        if (axis.norm() == 0) {
            throw std::runtime_error("start mid end center are on one same line, cannot calculate axis");
        }
    }

    if (mid_pos_ptr && positive) {
        Eigen::Vector3d cm_dir = (*mid_pos_ptr - center_pos).normalized();
        double se = std::acos(cs_dir.dot(ce_dir));
        double sm = std::acos(cs_dir.dot(cm_dir));
        double me = std::acos(cm_dir.dot(ce_dir));
        if (se - sm - me < -0.001) {
            std::cout << "arc angle greater than 180 degree\n";
            axis = -axis;
        }
    }

    return (positive ? 1 : -1) * axis;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> calArcCenterAxis(const Eigen::Vector3d& start_pos,
                                                              const Eigen::Vector3d& end_pos,
                                                              const Eigen::Vector3d center_pos_hint,
                                                              bool positive) {
    Eigen::Vector3d axis = calArcAxis(start_pos, end_pos, center_pos_hint, positive);

    Eigen::Vector3d mid = (start_pos + end_pos) / 2;
    Eigen::Vector3d se_dir = (end_pos - start_pos).normalized();

    Eigen::Vector3d mid_center_dir = axis.cross(se_dir);
    Eigen::Vector3d mid_center_hint = center_pos_hint - mid;

    Eigen::Vector3d center = (mid_center_hint.dot(mid_center_dir)) * mid_center_dir + mid;
    return {center, axis};
}

Eigen::Vector3d calTrajAxisX(const std::vector<std::vector<double>>& traj, int index, int window_size) {
    Eigen::Vector3d axis_x;
    int left = window_size / 2;
    int right = window_size - left;

    int ind_left;
    int ind_right;

    if (index - left < 0) {
        ind_left = 0;
        ind_right = std::min(window_size + ind_left, static_cast<int>(traj.size() - 1));
    } else if (index + right >= static_cast<int>(traj.size())) {
        ind_right = static_cast<int>(traj.size()) - 1;
        ind_left = std::max(0, ind_right - window_size);
    } else {
        ind_left = index - left;
        ind_right = index + right;
    }

    for (int i = 0; i < 3; i++) {
        axis_x[i] = (traj[ind_right][i + 1] - traj[ind_left][i + 1]) / window_size;
    }

    axis_x.normalize();
    return axis_x;
}

Eigen::Vector3d calTransTrajAxisX(const std::vector<std::vector<double>>& traj, int index, int window_size) {
    Eigen::Vector3d axis_x;
    int left = window_size / 2;
    int right = window_size - left;

    int ind_left;
    int ind_right;

    if (index - left < 0) {
        ind_left = 0;
        ind_right = std::min(window_size + ind_left, static_cast<int>(traj[0].size() - 1));
    } else if (index + right >= static_cast<int>(traj[0].size())) {
        ind_right = static_cast<int>(traj[0].size()) - 1;
        ind_left = std::max(0, ind_right - window_size);
    } else {
        ind_left = index - left;
        ind_right = index + right;
    }

    for (int i = 0; i < 3; i++) {
        axis_x[i] = (traj[i + 1][ind_right] - traj[i + 1][ind_left]) / window_size;
    }

    axis_x.normalize();
    return axis_x;
}

}  // namespace vp::tp
