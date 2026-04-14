/**
 * @file CircleTrajectory.cpp
 * @brief 圆弧轨迹规划
 * @version 0.1
 * @date 2023-11-16
 *
 * @copyright Copyright (c) 2023 Original Source
 */

#include "CircleTrajectory.h"
#include <spdlog/spdlog.h>
#include <iostream>
#include "TrapezoidalVelocityTrajectory.h"
#include "motionTransform.h"

#ifdef USE_MSVC
#define _USE_MATH_DEFINES
#include <math.h>
#endif

namespace vp::tp {
    std::vector<double> getTcpPoseFromPositionQuaternion(Eigen::Vector3d pos, Eigen::Quaterniond q) {
        Eigen::AngleAxisd aa(q);
        Eigen::Vector3d aa_vec = aa.axis();
        aa_vec *= aa.angle();

        return {pos[0], pos[1], pos[2], aa_vec.x(), aa_vec.y(), aa_vec.z()};
    }

    CircleTrajectory::~CircleTrajectory() = default;

    CircleTrajectory::CircleTrajectory(std::vector<double> _start_pose, std::vector<double> _center_pos,
                                       bool is_linear) {
        if (_start_pose.size() != 6) {
            spdlog::error("_start_pose must be equal to 6");
            return;
        }
        if (_center_pos.size() != 3) {
            spdlog::error("_center_pos must be equal to 3");
            return;
        }

        m_start_pose = Eigen::Map<Eigen::Matrix<double, 6, 1>>(_start_pose.data());
        m_start_pos = m_start_pose.head(3);

        m_center_pos = Eigen::Map<Eigen::Matrix<double, 3, 1>>(_center_pos.data());

        // 由圆心指向起点向量
        Eigen::Vector3d start_vec = (m_start_pos - m_center_pos);
        m_radius = start_vec.norm();

        // 设置规划类型变量
        if (is_linear)
            m_plan_type = PlanType::START_CENTER_AXIS_LINEAR;
        else
            m_plan_type = PlanType::START_CENTER_AXIS_ANGULAR;
    }

    CircleTrajectory::CircleTrajectory(std::vector<double> _start_pose, std::vector<double> _end_pose,
                                       std::vector<double> _center_pos) {
        if (_start_pose.size() != 6) {
            spdlog::error("_start_pose must be equal to 6");
            return;
        }
        if (_end_pose.size() != 6) {
            spdlog::error("_end_pose must be equal to 6");
            return;
        }
        if (_center_pos.size() != 3) {
            spdlog::error("_center_pos must be equal to 3");
            return;
        }

        m_start_pose = Eigen::Map<Eigen::Matrix<double, 6, 1>>(_start_pose.data());
        m_start_pos = m_start_pose.head(3);

        m_end_pose = Eigen::Map<Eigen::Matrix<double, 6, 1>>(_end_pose.data());
        m_end_pos = m_end_pose.head(3);

        m_center_pos = Eigen::Map<Eigen::Matrix<double, 3, 1>>(_center_pos.data());

        // 由圆心指向起点向量
        Eigen::Vector3d start_vec = (m_start_pos - m_center_pos);
        m_radius = start_vec.norm();

        m_plan_type = PlanType::START_CENTER_END_LINEAR;
    }

    void CircleTrajectory::set_timeinterval(double _time_interval) {
        if (_time_interval <= 0) {
            spdlog::error("_time_interval must be greater than 0");
            return;
        }

        m_time_interval = _time_interval;
    }

    void CircleTrajectory::cal_trapezoidal_vel_trajectory_help_function(std::vector<std::vector<double>>& trajectory,
                                                                        double traj_length, double _v_max,
                                                                        double _a_max) {
        // 由圆心指向起点向量
        Eigen::Vector3d start_vec = (m_start_pos - m_center_pos);

        Eigen::Quaterniond qf = m_start_q, qt = m_end_q, q;

        // 梯形速度轨迹规划结果数据
        std::pair<std::vector<double>, std::vector<double>> p_and_v =
            TrapezoidalVelocityTrajectory::normalization_factor(0, traj_length, 0, 0, _v_max, _a_max, -_a_max, 0.0,
                                                                m_time_interval);

        std::vector<double> lambda = p_and_v.first;  // 位置的归一化

        /// @cond 轨迹计算临时变量
        // 轨迹点姿态，轴角表示
        Eigen::AngleAxisd aa;
        // 上一轨迹点姿态，轴角表示，用于计算速度
        Eigen::AngleAxisd aa_pre;
        // 轨迹点姿态，轴×角表示
        Eigen::Vector3d aa_vec;

        Eigen::Vector3d orientation_vel;

        // 轨迹点时间、位姿、速度数据（时间 + 6维位置 + 6维速度）
        std::vector<double> pv(13, 0);
        // 轨迹点位姿数据（6维位置）
        std::vector<double> last_p(6, 0);

        /// @endcond

        for (int i = 0; i < lambda.size(); i++) {
            // 起点至当前轨迹点部分轨迹占整体轨迹比例
            double ratio = lambda[i];
            // 起点至当前轨迹点对应圆心角
            double theta = ratio * m_theta;

            // 起点至当前轨迹点对应轴角变换
            Eigen::AngleAxisd t_rv(theta, m_axis);

            // 计算当前轨迹点位置
            Eigen::Vector3d pos = (t_rv.toRotationMatrix() * start_vec) + m_center_pos;

            // 计算当前轨迹点姿态
            if (m_plan_type == PlanType::START_CENTER_END_LINEAR)
                aa = qf.slerp(ratio, qt);
            else {
                aa = t_rv * qf;
            }
            aa_vec = aa.angle() * aa.axis();

            pv = {i * m_time_interval, pos[0], pos[1], pos[2], aa_vec[0], aa_vec[1], aa_vec[2], 0, 0, 0, 0, 0, 0};

            // 计算轨迹点速度
            if (i != 0) {
                // 位置各维分别差分
                for (int j = 0; j < 3; j++) {
                    pv[j + 7] = (pv[j + 1] - last_p[j]) / m_time_interval;
                }

                // 姿态按轴角变化计算
                orientation_vel = vp::math::anguler_velocity_from_angleAxis(aa_pre, aa, m_time_interval);
                for (int j = 0; j < 3; j++) {
                    pv[j + 10] = orientation_vel[j];
                }
            }

            aa_pre = aa;

            std::copy(pv.begin() + 1, pv.begin() + 7, last_p.begin());

            trajectory.push_back(pv);
        }

        // 轨迹终点
        pv = {lambda.size() * m_time_interval,
              m_end_pose[0],
              m_end_pose[1],
              m_end_pose[2],
              m_end_pose[3],
              m_end_pose[4],
              m_end_pose[5],
              0,
              0,
              0,
              0,
              0,
              0};
        trajectory.push_back(pv);

        std::cout << " start " << trajectory[0][0] << " " << trajectory[0][1] << " " << trajectory[0][2] << std::endl;
        std::cout << " "
                  << " end " << trajectory[lambda.size() - 1][0] << " " << trajectory[lambda.size() - 1][1] << " "
                  << trajectory[lambda.size() - 1][2] << std::endl;
    }

    std::vector<std::vector<double>> CircleTrajectory::cal_trapezoidal_vel_trajectory(double _v_max, double _a_max,
                                                                                      Eigen::Vector3d axis,
                                                                                      double theta) {
        // 检查规划类型是否与构造函数数据匹配
        if (m_plan_type != PlanType::START_CENTER_AXIS_LINEAR && m_plan_type != PlanType::START_CENTER_AXIS_ANGULAR) {
            spdlog::error(
                "plan type of this function is not mathcing with the Construct "
                "function");
            return {};
        }

        // 由圆心指向起点向量
        Eigen::Vector3d start_vec = (m_start_pos - m_center_pos);
        m_axis = axis.normalized();
        m_theta = theta;

        // 轴角表示起点至终点的旋转变化
        Eigen::AngleAxisd rv(m_theta, m_axis);
        // 计算终点位置。相对定系（基坐标系）旋转，左乘
        m_end_pos = (rv.toRotationMatrix() * start_vec) + m_center_pos;

        // 计算起点姿态和终点姿态，四元数表示
        m_start_q = vp::math::getQuaternionFromTcpPose(m_start_pose);
        m_end_q = rv * m_start_q;

        {
            std::vector<double> _end_pose = getTcpPoseFromPositionQuaternion(m_end_pos, m_end_q);
            m_end_pose << Eigen::Map<Eigen::Matrix<double, 6, 1>>(_end_pose.data());
        }

        // 计算轨迹长度，若线速度规划，则以弧长作为轨迹长度；若角速度规划，则以弧度作为轨迹长度
        double traj_length = std::fabs(m_theta) * ((m_plan_type == PlanType::START_CENTER_AXIS_LINEAR) ? m_radius : 1);

        // 圆弧轨迹规划结果
        std::vector<std::vector<double>> trajectory;

        cal_trapezoidal_vel_trajectory_help_function(trajectory, traj_length, _v_max, _a_max);

        return trajectory;
    }

    std::vector<std::vector<double>> CircleTrajectory::cal_trapezoidal_vel_trajectory(double _v_max, double _a_max,
                                                                                      bool positive) {
        // 检查规划类型是否与构造函数数据匹配
        if (m_plan_type != PlanType::START_CENTER_END_LINEAR) {
            spdlog::error(
                "plan type of this function is not mathcing with the Construct "
                "function");
            return {};
        }

        // 由圆心指向起点向量
        Eigen::Vector3d start_vec = (m_start_pos - m_center_pos);
        // 由圆心指向终点向量
        Eigen::Vector3d end_vec = (m_end_pos - m_center_pos);

        // 计算旋转轴和角度
        // 旋转轴方向矢量
        Eigen::Vector3d axis = start_vec.cross(end_vec);
        // 旋转角度
        double theta = std::acos(start_vec.normalized().dot(end_vec.normalized()));
        m_axis = axis.normalized();
        m_theta = theta;

        // 根据正反转标志位改变旋转轴方向和角度大小
        if (!positive) {
            m_axis *= -1;
            m_theta = 2 * M_PI - m_theta;
        }

        // 轴角表示起点至终点的旋转变化
        Eigen::AngleAxisd rv(m_theta, m_axis);

        // 起点姿态和终点姿态，四元数表示
        m_start_q = vp::math::getQuaternionFromTcpPose(m_start_pose);
        m_end_q = vp::math::getQuaternionFromTcpPose(m_end_pose);

        // 轨迹长度，弧长表示
        double traj_length = m_radius * std::fabs(m_theta);

        // 圆弧轨迹规划结果
        std::vector<std::vector<double>> trajectory;

        cal_trapezoidal_vel_trajectory_help_function(trajectory, traj_length, _v_max, _a_max);

        return trajectory;
    }
}  // namespace vp::tp
