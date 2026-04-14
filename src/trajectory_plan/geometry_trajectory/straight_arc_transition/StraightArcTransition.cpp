//
// Created by hengjinli on 23-7-25.
//

#include "StraightArcTransition.h"
#include <iostream>
#include "TrapezoidalVelocityTrajectory.h"

StraightArcTransition::StraightArcTransition()
    : m_vv(.005), m_time_interval(0.001), m_circle_initial_pose(Eigen::Matrix3d::Identity()), m_a(0.1) {}

StraightArcTransition::~StraightArcTransition() {}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> StraightArcTransition::segmented_path_transfer(
    const Eigen::Vector3d& _point0, const Eigen::Vector3d& _point1, const Eigen::Vector3d& _point2, double _r) {
    double L_p1p0 = (_point0 - _point1).norm();  // 矢量长度
    double L_p1p2 = (_point1 - _point2).norm();

    Eigen::Vector3d vec_p1p0 = _point0 - _point1;
    Eigen::Vector3d vec_p1p2 = _point2 - _point1;
    double theta = acos((vec_p1p0.dot(vec_p1p2)) / (L_p1p0 * L_p1p2));

    // 求转接点Pt1、pt2
    Eigen::Vector3d vec_p1pt1 = (_r / std::tan(theta / 2) / L_p1p0) * vec_p1p0;
    Eigen::Vector3d vec_p1pt2 = (_r / std::tan(theta / 2) / L_p1p2) * vec_p1p2;
    Eigen::Vector3d pt1 = _point1 + vec_p1pt1;
    Eigen::Vector3d pt2 = _point1 + vec_p1pt2;

    // 求路径长度d1、弧长d2
    //    double d1 = (pt1 - _point0).norm();
    //    double d2 = (M_PI - theta) * r;

    Eigen::Vector3d vec_pt1M = .5 * (pt2 - pt1);
    Eigen::Vector3d M = pt1 + vec_pt1M;
    Eigen::Vector3d vec_p1M = M - _point1;
    double p1M = vec_p1M.norm();
    double p1C = _r / sin(theta / 2);
    Eigen::Vector3d vec_p1C = (p1C / p1M) * vec_p1M;
    Eigen::Vector3d C = _point1 + vec_p1C;

    return std::make_tuple(pt1, pt2, C);
}

std::vector<std::array<double, 6>> StraightArcTransition::calculate_transition_arc(std::array<double, 3> _point0,
                                                                                   std::array<double, 3> _point1,
                                                                                   std::array<double, 3> _point2,
                                                                                   double r) {
    auto point0 = Eigen::Map<const Eigen::Vector3d>(_point0.data());
    auto point1 = Eigen::Map<const Eigen::Vector3d>(_point1.data());
    auto point2 = Eigen::Map<const Eigen::Vector3d>(_point2.data());

    auto path_info = segmented_path_transfer(point0, point1, point2, r);  // 计算圆弧过渡点
    auto pt1 = std::get<0>(path_info);                                    // 过渡点1
    auto pt2 = std::get<1>(path_info);                                    // 过渡点2
    auto center = std::get<2>(path_info);                                 // 圆心

    m_circle_initial_pose = calculate_initial_pose(pt1, pt2, center);

    std::vector<std::array<double, 6>> trajectory;
    // 直线段
    Eigen::MatrixXd::Index max_index;
    (point0 - pt1).cwiseAbs().maxCoeff(&max_index);  // 确定XYZ轴上哪个路程最大

    TrapezoidalVelocityTrajectory velocityTrajectory;
    std::pair<std::vector<double>, std::vector<double>> p_and_v = velocityTrajectory.normalization_factor(
        point0[max_index], pt1[max_index], 0.0, m_vv, m_vv, m_a, -m_a, 0.0, m_time_interval);

    std::vector<double> lambda = p_and_v.first;  // 位置的归一化因子

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner(3, 3) = m_circle_initial_pose;
    Eigen::AngleAxisd axis_angle(m_circle_initial_pose);
    std::array<double, 6> T_temp{};
    std::array<double, 3> P{};
    std::array<double, 3> R{};
    Eigen::Vector3d position_current;
    Eigen::Vector3d pose_current;
    for (double i : lambda) {
        position_current = point0 + i * (pt1 - point0);
        pose_current = axis_angle.axis() * axis_angle.angle();
        Eigen::Matrix<double, 3, 1>::Map(&P[0], position_current.rows(), position_current.cols()) = position_current;
        Eigen::Matrix<double, 3, 1>::Map(&R[0], pose_current.rows(), pose_current.cols()) = pose_current;
        trajectory.push_back({P[0], P[1], P[2], R[0], R[1], R[2]});
    }

    // 过渡圆弧
    Eigen::Vector3d v_normal = calculate_plane_normal_vector(pt1, pt2, center);

    double theta = calculate_angle(pt1, pt2, center);  // 计算过渡圆弧的角度
    double v_sum = m_vv * (point0 - pt1).norm() / abs(point0[max_index] - pt1[max_index]);
    double omega = v_sum / r;              // 计算角速度
    double time = theta / omega;           // 时间
    size_t step = time / m_time_interval;  // 步数

    Eigen::Vector3d initial_radius_vector = pt1 - center;  // 初始的半径向量
    for (int i = 0; i < step; ++i) {
        double theta_current = omega * i / static_cast<double>(step) * time;
        auto circle_motion_data = calculate_circular_arc_pose(v_normal, initial_radius_vector, center, theta_current);
        position_current = circle_motion_data.first;
        pose_current = circle_motion_data.second;
        Eigen::Matrix<double, 3, 1>::Map(&P[0], position_current.rows(), position_current.cols()) = position_current;
        Eigen::Matrix<double, 3, 1>::Map(&R[0], pose_current.rows(), pose_current.cols()) = pose_current;
        trajectory.push_back({P[0], P[1], P[2], R[0], R[1], R[2]});
    }

    // 直线段
    (point2 - pt2).cwiseAbs().maxCoeff(&max_index);  // 确定XYZ轴上哪个路程最大
    double v_component = v_sum / (point2 - pt2).norm() * abs(point2[max_index] - pt2[max_index]);
    p_and_v = velocityTrajectory.normalization_factor(pt2[max_index], point2[max_index], v_component, 0.0, v_component,
                                                      m_a, -m_a, 0.0, m_time_interval);

    lambda = p_and_v.first;

    Eigen::Matrix<double, 6, 1> circle_last_pose = Eigen::Map<Eigen::Matrix<double, 6, 1>>(trajectory.back().data());
    int j = 0;
    for (double i : lambda) {
        position_current = pt2 + i * (point2 - pt2);
        pose_current = circle_last_pose.tail(3);
        Eigen::Matrix<double, 3, 1>::Map(&P[0], position_current.rows(), position_current.cols()) = position_current;
        Eigen::Matrix<double, 3, 1>::Map(&R[0], pose_current.rows(), pose_current.cols()) = pose_current;
        trajectory.push_back({P[0], P[1], P[2], R[0], R[1], p_and_v.second[++j]});
    }

    return trajectory;
}

Eigen::Matrix3d StraightArcTransition::calculate_initial_pose(const Eigen::Vector3d& _point0,
                                                              const Eigen::Vector3d& _point1,
                                                              const Eigen::Vector3d& _center) {
    Eigen::Vector3d v_normal = calculate_plane_normal_vector(_point0, _point1, _center);

    Eigen::Vector3d initial_radius_vector = _point0 - _center;

    Eigen::Vector3d U = -v_normal;
    Eigen::Vector3d W = initial_radius_vector.normalized();
    Eigen::Vector3d V = W.cross(U);

    Eigen::Matrix3d initial_pose;
    initial_pose.col(0) = U;
    initial_pose.col(1) = V;
    initial_pose.col(2) = W;

    return initial_pose;
}

Eigen::Vector3d StraightArcTransition::calculate_plane_normal_vector(const Eigen::Vector3d& _point0,
                                                                     const Eigen::Vector3d& _point1,
                                                                     const Eigen::Vector3d& _center) {
    // 圆心指向两个端点的矢量
    Eigen::Vector3d v1 = _point0 - _center;
    Eigen::Vector3d v2 = _point1 - _center;

    Eigen::Vector3d v_normal = v1.cross(v2);

    v_normal = v_normal.normalized();

    return v_normal;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> StraightArcTransition::calculate_circular_arc_pose(
    const Eigen::Vector3d& _axis, Eigen::Vector3d _initial_radius_vector, Eigen::Vector3d _center,
    double _theta_current) {
    Eigen::AngleAxisd rotate_vector(_theta_current, _axis);
    Eigen::Vector3d current_radius_vector =
        rotate_vector.toRotationMatrix() * _initial_radius_vector;  // 使半径向量绕旋转轴旋转
    Eigen::Vector3d p_current = _center + current_radius_vector;    // 轨迹点坐标

    rotate_vector.fromRotationMatrix(rotate_vector.toRotationMatrix() * m_circle_initial_pose);

    return std::make_pair(p_current, rotate_vector.angle() * rotate_vector.axis());
}

double StraightArcTransition::calculate_angle(const Eigen::Vector3d& _point0, const Eigen::Vector3d& _point1,
                                              const Eigen::Vector3d& _center) const {
    Eigen::Vector3d v1_normalized = (_point0 - _center).normalized();
    Eigen::Vector3d v2_normalized = (_point1 - _center).normalized();

    // 计算旋转角度
    double theta = acos(v1_normalized.dot(v2_normalized));
    return theta;
}

void StraightArcTransition::set_time_interval(double _time) {
    m_time_interval = _time;
}

void StraightArcTransition::set_v_max(double _v_max) {
    m_vv = _v_max;
}

void StraightArcTransition::set_a_max(double _a_max) {
    m_a = _a_max;
}
