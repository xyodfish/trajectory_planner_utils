/**
 * @file Algorithm.cpp
 * @author original.author@example.com
 * @brief 轨迹相关功能
 * @version 0.1
 * @date 2023-10-23
 *
 * @copyright Copyright (c) 2023 Original Source
 */

#include "traj.h"
#include <yaml-cpp/yaml.h>
#include <iomanip>
#include <iostream>
#include <list>
#include "totg_Trajectory.h"

namespace vp::tp {
    bool getTrajL(const std::vector<std::vector<double>>& path, const std::string& config_file,
                  std::vector<TrajPoint>& traj, double time_step) {
        // 内部包括各维度最大速度与加速度及允许最大轨迹与期望路径差异（加权求和后），各维度权重
        YAML::Node config = YAML::LoadFile(config_file);
        Eigen::VectorXd maxAcceleration(6);
        Eigen::VectorXd maxVelocity(6);
        Eigen::VectorXd weight(6);
        for (unsigned int i = 0; i < maxAcceleration.size(); i++) {
            weight[i] = config["weight"][i].as<double>();
            maxAcceleration[i] = config["max_acceleration"][i].as<double>() * weight[i];
            maxVelocity[i] = config["max_velocity"][i].as<double>() * weight[i];
        }

        double maxDeviation;
        maxDeviation = config["max_deviation"].as<double>();

        std::list<Eigen::VectorXd> waypoints;
        Eigen::VectorXd waypoint(3);

        for (const auto& p : path) {
            for (unsigned int i = 0; i < 6; i++) {
                waypoint[i] = p[i] * weight[i];
            }
            waypoints.push_back(waypoint);
        }

        TOTG::Trajectory trajectory(TOTG::Path(waypoints, maxDeviation), maxVelocity, maxAcceleration);
        TrajPoint tp;
        tp.pos.resize(6);
        tp.vel.resize(6);

        if (trajectory.isValid()) {
            traj.clear();

            double duration = trajectory.getDuration();
            for (double t = 0.0; t < duration; t += time_step) {
                tp.time = t;
                tp.pos = trajectory.getPosition(t);
                tp.vel = trajectory.getVelocity(t);

                for (unsigned int i = 0; i < 6; i++) {
                    tp.pos[i] /= weight[i];
                    tp.vel[i] /= weight[i];
                }

                traj.push_back(tp);
            }

            return true;
        } else {
            std::cout << "Trajectory generation failed." << '\n';
            return false;
        }
    }

    void readTrajL(std::vector<TrajPoint>& traj, const std::string& file_name) {
        unsigned int dim = 6;

        traj.clear();

        std::ifstream f;
        f.open(file_name);

        Eigen::VectorXd pos(dim);
        Eigen::VectorXd vel(dim);
        TrajPoint tp_temp;

        while (f.peek() != EOF) {
            f >> tp_temp.time;

            for (unsigned int i = 0; i < dim; i++) {
                f >> pos[i];
            }

            for (unsigned int i = 0; i < dim; i++) {
                f >> vel[i];
            }

            tp_temp.pos = pos;
            tp_temp.vel = vel;

            traj.push_back(tp_temp);
        }
    }
    void writeTrajL(const std::vector<TrajPoint>& traj, std::string file, const std::string& separator) {
        unsigned int num = traj.size();
        unsigned int dim = traj[0].pos.size();

        std::ofstream f;
        f.open(file);

        for (unsigned int i = 0; i < num; i++) {
            for (unsigned int j = 0; j < dim; j++) {
                f << traj[i].pos[j] << separator;
            }

            for (unsigned int j = 0; j < dim; j++) {
                f << traj[i].vel[j];
                if (j != dim - 1) {
                    f << separator;
                }
            }

            if (i != num - 1) {
                f << '\n';
            }
        }

        f.close();
    }

    Eigen::Vector3d calArcCenter(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos, Eigen::Vector3d axis,
                                 double radius, bool smaller_than_pi) {
        // mid point of start  and end
        Eigen::Vector3d mid = (start_pos + end_pos) / 2;
        // half distance of start to end
        double se_2 = (end_pos - start_pos).norm() / 2;
        // length from mid point to center
        double a = std::sqrt(std::pow(radius, 2) - std::pow(se_2, 2));

        // direction of start to end
        Eigen::Vector3d se_dir = (end_pos - start_pos) / se_2 / 2;

        axis.normalize();

        Eigen::Vector3d mid_center_dir = axis.cross(se_dir);

        return mid + (a * mid_center_dir) * (smaller_than_pi ? 1 : -1);
    }

    Eigen::Vector3d calArcAxis(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos,
                               const Eigen::Vector3d& center_pos, bool positive, const Eigen::Vector3d* mid_pos_ptr) {
        // 由圆心指向圆弧起点的方向矢量
        Eigen::Vector3d cs_dir = (start_pos - center_pos).normalized();
        // 由圆心指向圆弧终点的方向矢量
        Eigen::Vector3d ce_dir = (end_pos - center_pos).normalized();
        Eigen::Vector3d axis = (cs_dir.cross(ce_dir)).normalized();

        if (axis.norm() == 0) {
            if (mid_pos_ptr == nullptr) {
                throw(std::runtime_error("start end center are on one same line, cannot calculate axis"));
            } else {
                Eigen::Vector3d mid_pos = *mid_pos_ptr;
                if (!(mid_pos[0] == end_pos[0] && mid_pos[1] == end_pos[1] && mid_pos[2] == end_pos[2])) {

                    Eigen::Vector3d cm_dir = (mid_pos - center_pos).normalized();
                    axis = (cs_dir.cross(cm_dir)).normalized();
                }

                if (axis.norm() == 0) {
                    throw(std::runtime_error("start mid end center are on one same line, cannot calculate axis"));
                }
            }
        }

        if (mid_pos_ptr && positive) {
            Eigen::Vector3d cm_dir = (*mid_pos_ptr - center_pos).normalized();
            Eigen::Vector3d axis_2 = (cm_dir.cross(ce_dir)).normalized();
            double se = acos(cs_dir.dot(ce_dir));
            double sm = acos(cs_dir.dot(cm_dir));
            double me = acos(cm_dir.dot(ce_dir));
            if (se - sm - me < -0.001) {
                std::cout << "arc angle greater than 180 degree\n";
                axis = -axis;
            }
        }

        return (positive ? 1 : -1) * axis;
    }

    std::pair<Eigen::Vector3d, Eigen::Vector3d> calArcCenterAxis(const Eigen::Vector3d& start_pos,
                                                                 const Eigen::Vector3d& end_pos,
                                                                 const Eigen::Vector3d center_pos_hint, bool positive) {
        Eigen::Vector3d axis = calArcAxis(start_pos, end_pos, center_pos_hint, positive);

        // mid point of start  and end
        Eigen::Vector3d mid = (start_pos + end_pos) / 2;
        // direction of start to end
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
            ind_right = std::min(window_size + ind_left, (int)(traj.size() - 1));
        } else if (index + right >= (int)(traj.size())) {
            ind_right = traj.size() - 1;
            ind_left = std::max(0, ind_right - window_size);
        } else {
            ind_left = index - left;
            ind_right = index + right;
        }

        for (int i = 0; i < 3; i++)
            axis_x[i] = (traj[ind_right][i + 1] - traj[ind_left][i + 1]) / window_size;

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
            ind_right = std::min(window_size + ind_left, (int)(traj[0].size() - 1));
        } else if (index + right >= (int)(traj[0].size())) {
            ind_right = traj[0].size() - 1;
            ind_left = std::max(0, ind_right - window_size);
        } else {
            ind_left = index - left;
            ind_right = index + right;
        }

        for (int i = 0; i < 3; i++)
            axis_x[i] = (traj[i + 1][ind_right] - traj[i + 1][ind_left]) / window_size;

        axis_x.normalize();

        return axis_x;
    }
}  // namespace vp::tp
// namespace vp::tp
