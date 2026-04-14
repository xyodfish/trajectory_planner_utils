//
// Created by hengjinli on 23-8-3.
//

#include "EllipseTrajectory.h"
#include <iostream>
#include "TrapezoidalVelocityTrajectory.h"
#include "dbg.h"
#include "motionTransform.h"

#ifdef USE_MSVC
#define _USE_MATH_DEFINES
#include <math.h>
#endif

using namespace vp::tp;

EllipseTrajectory::~EllipseTrajectory() = default;

void EllipseTrajectory::set_start_end_pose(std::array<double, 6> _start_pose, std::array<double, 6> _end_pose) {
    m_start_pose = Eigen::Map<Eigen::Matrix<double, 6, 1>>(_start_pose.data());
    m_end_pose   = Eigen::Map<Eigen::Matrix<double, 6, 1>>(_end_pose.data());
}

EllipseTrajectory::EllipseTrajectory(std::array<double, 6> _start_pose, std::array<double, 6> _end_pose)
    : m_start_pose(Eigen::Map<Eigen::Matrix<double, 6, 1>>(_start_pose.data())),
      m_end_pose(Eigen::Map<Eigen::Matrix<double, 6, 1>>(_end_pose.data())) {}

EllipseTrajectory::EllipseTrajectory(std::vector<double> _start_pose, std::vector<double> _end_pose)
    : m_start_pose(Eigen::Map<Eigen::Matrix<double, 6, 1>>(_start_pose.data())),
      m_end_pose(Eigen::Map<Eigen::Matrix<double, 6, 1>>(_end_pose.data())) {
    start_q = vp::math::getQuaternionFromTcpPose(_start_pose);
    end_q   = vp::math::getQuaternionFromTcpPose(_end_pose);
}

double EllipseTrajectory::calculate_curvature_radius(double a, double b, double t) {
    double aa     = a * a;
    double bb     = b * b;
    double sint_2 = std::pow(std::sin(t), 2);
    double cost_2 = std::pow(std::cos(t), 2);

    double rho = std::pow(aa * sint_2 + bb * cost_2, 1.5);
    rho /= (a * b);

    return rho;
}

double EllipseTrajectory::calculate_curvature_radius(double a, double b, double x, double y) {
    double a4 = std::pow(a, 4);
    double b4 = std::pow(b, 4);
    double x2 = std::pow(x, 2);
    double y2 = std::pow(y, 2);

    double rho = std::pow((a4 * y2 + b4 * x2), 1.5) / (a4 * b4);
    return rho;
}
double EllipseTrajectory::calculate_perimeter(double a, double b) {
    double lambda   = (a - b) / (a + b);
    double lam_pow2 = std::pow(lambda, 2);
    double temp     = 3 * lam_pow2 / (10 + std::sqrt(4.0 - 3 * lam_pow2));
    return M_PI * (a + b) * (1 + temp);
}

std::pair<double, double> EllipseTrajectory::calculate_curve_circle_center(double a, double b, double c, double t) {
    std::pair<double, double> circleCenter;
    circleCenter.first  = (c * c / a) * (std::pow((std::cos(t)), 3));
    circleCenter.second = -(c * c / b) * (std::pow(std::sin(t), 3));

    return circleCenter;
}

std::vector<std::array<double, 6>> EllipseTrajectory::cal_trapezoidal_rad_vel_trajectory(double _v_max, double _a_max,
                                                                                         EllipseTrajType _traj_type) {
    Eigen::Vector3d point_start = m_start_pose.head(3);
    Eigen::Vector3d point_end   = m_end_pose.head(3);

    Eigen::Vector3d vec = point_start - point_end;

    // 长短半轴计算
    double semimajor_axis = vec.head(2).norm();  // 半长轴
    double semiminor_axis = abs(vec(2));         // 半短轴

    // 椭圆中心
    Eigen::Vector3d center;
    if (_traj_type == insert_trajectory) {
        center << point_end[0], point_end[1], point_start[2];
    } else {
        center << point_start[0], point_start[1], point_end[2];
    }

    // 椭圆中心坐标系姿态
    Eigen::Vector3d U = point_start - center;
    U.normalize();
    Eigen::Vector3d V = point_end - center;
    V.normalize();
    Eigen::Vector3d W = U.cross(V);

    Eigen::Matrix3d R;  // 椭圆中心坐标系姿态
    R.col(0) = U;
    R.col(1) = V;
    R.col(2) = W;

    // std::pair< std::vector< double >, std::vector< double > > p_and_v =
    //     TrapezoidalVelocityTrajectory::normalization_factor(
    //         0, M_PI_2, 0, 0, _v_max, _a_max, -_a_max, 0.0, m_time_interval);

    // std::vector< double > lambda = p_and_v.first; // 位置的归一化

    std::vector<double> lambda;

    Eigen::Vector3d ellipse_point;
    Eigen::Vector3d ellipse_point_wrt_base;
    Eigen::Matrix3d R_point;
    std::vector<std::array<double, 6>> trajectory;

    for (double i : lambda) {
        double t = 0 + (1 - i) * (M_PI_2 - 0);  // 参数方程的变量 t
        double x = semimajor_axis * cos(t);     // 参数方程
        double y = semiminor_axis * sin(t);     // 参数方程

        ellipse_point << x, y, 0.0;                            // 局部坐标系下的点
        ellipse_point_wrt_base << R * ellipse_point + center;  // 基坐标系下的点

        // 椭圆上点的跟随坐标系的X轴和Y轴
        double x_dot = -semimajor_axis * sin(t);  // x对t求导
        double y_dot = semiminor_axis * cos(t);   // y对t求导

        Eigen::Vector3d V_point = W;
        Eigen::Vector3d W_point = R * Eigen::Vector3d({x_dot, y_dot, 0});
        W_point.normalize();
        Eigen::Vector3d U_point = V_point.cross(W_point);
        R_point << U_point, V_point, W_point;

        Eigen::AngleAxisd angleAxisTransformation(R_point);

        trajectory.push_back({ellipse_point_wrt_base[0], ellipse_point_wrt_base[1], ellipse_point_wrt_base[2],
                              angleAxisTransformation.axis()[0] * angleAxisTransformation.angle(),
                              angleAxisTransformation.axis()[1] * angleAxisTransformation.angle(),
                              angleAxisTransformation.axis()[2] * angleAxisTransformation.angle()});
    }

    return trajectory;
}

std::pair<double, double> EllipseTrajectory::calculate_next_ellipse_xy(double deltaTheta,
                                                                       std::pair<double, double> point,
                                                                       std::pair<double, double> center) {
    //绕曲率圆圆心旋转
    auto cos_dt = std::cos(deltaTheta);
    auto sin_dt = std::sin(deltaTheta);

    auto nextX =
        cos_dt * point.first - sin_dt * point.second - center.first * cos_dt + center.second * sin_dt + center.first;
    auto nextY =
        sin_dt * point.first + cos_dt * point.second - center.first * sin_dt - center.second * cos_dt + center.second;

    return std::make_pair(nextX, nextY);
}

std::pair<double, double> EllipseTrajectory::calculate_this_ellipse_point(double a, double b, double theta) {
    double x = a * std::cos(theta);
    double y = b * std::sin(theta);

    return std::make_pair(x, y);
}

std::pair<double, double> EllipseTrajectory::calculate_next_ellipse_point(double a, double b, double c, double& theta,
                                                                          double arcLen) {
    std::pair<double, double> this_point       = calculate_this_ellipse_point(a, b, theta);
    double curvRadius                          = calculate_curvature_radius(a, b, this_point.first, this_point.second);
    std::pair<double, double> curvCircleCenter = calculate_curve_circle_center(a, b, c, theta);

    //旋转角度步长 约等于 给定的弧长s / 曲率半径rho
    double deltaTheta = arcLen / curvRadius;

    std::pair<double, double> nextXY = calculate_next_ellipse_xy(deltaTheta, this_point, curvCircleCenter);

    theta = std::atan2(a / b * nextXY.second, nextXY.first);

    return calculate_this_ellipse_point(a, b, theta);
}

void EllipseTrajectory::set_velocity_planner(const std::vector<BCs<double>>& BCs, const std::string& algo) {
    set_boundary_conditions(BCs);
    set_planner_name(algo);
}

void EllipseTrajectory::set_boundary_conditions(const std::vector<BCs<double>>& BCs) {
    BCs_ = BCs;
}

void EllipseTrajectory::set_planner_name(const std::string& algo) {
    algo_ = algo;
}

std::vector<std::vector<double>> EllipseTrajectory::cal_trapezoidal_linear_vel_trajectory(double _v_max, double _a_max,
                                                                                          double _j_max,
                                                                                          EllipseTrajType _traj_type) {
#if 1
    double t = 0, deltaArcLen = 0.0, lastArcLen = 0.0;
    Eigen::Matrix3d R_point, R;
    Eigen::Vector3d ellipse_point, ellipse_point_wrt_base;
    Eigen::Vector3d center, U, V, W;

    std::vector<std::vector<double>> trajectory;
    std::pair<double, double> next_ellipse_point;

    bool if_reverse             = false;
    Eigen::Vector3d point_start = m_start_pose.head(3);
    Eigen::Vector3d point_end   = m_end_pose.head(3);

    Eigen::Vector3d vec = point_start - point_end;
    double d1 = vec.head(2).norm(), d2 = abs(vec(2));

    std::cout << "d1 is " << d1 << " d2 is " << d2 << std::endl;

    if (_traj_type == buckle_trajectory) {
        if (d1 > d2) {
            if (point_start(2) < point_end(2)) {
                center << point_end[0], point_end[1], point_start[2];
                U          = (center - point_start).normalized();
                V          = (center - point_end).normalized();
                if_reverse = false;
                t          = M_PI;

                std::cout << " buckle start(2) < end(2) d1 > d2" << std::endl;
            } else {
                center << point_start[0], point_start[1], point_end[2];
                U          = (center - point_end).normalized();
                V          = (center - point_start).normalized();
                if_reverse = false;
                t          = -M_PI;

                std::cout << " buckle start(2) > end(2) d1 > d2" << std::endl;
            }
        } else {
            if (point_start(2) < point_end(2)) {
                center << point_end[0], point_end[1], point_start[2];
                U          = (center - point_end).normalized();
                V          = (center - point_start).normalized();
                if_reverse = true;
                t          = M_PI;

                std::cout << " buckle start(2) < end(2) d1 < d2" << std::endl;
            } else {
                center << point_start[0], point_start[1], point_end[2];
                U          = (center - point_start).normalized();
                V          = (center - point_end).normalized();
                if_reverse = false;
                t          = M_PI;

                std::cout << " buckle start(2) > end(2) d1 < d2" << std::endl;
            }
        }
    } else {
        if (d1 > d2) {
            if (point_start(2) < point_end(2)) {
                center << point_start[0], point_start[1], point_end[2];
                U          = (center - point_end).normalized();
                V          = (center - point_start).normalized();
                if_reverse = false;
                t          = M_PI_2;

                std::cout << " insert start(2) < end(2) d1 > d2" << std::endl;
            } else {
                center << point_end[0], point_end[1], point_start[2];
                U          = (center - point_start).normalized();
                V          = (center - point_end).normalized();
                if_reverse = true;
                t          = M_PI_2;

                std::cout << " insert start(2) > end(2) d1 > d2" << std::endl;
            }
        } else {
            if (point_start(2) < point_end(2)) {
                center << point_start[0], point_start[1], point_end[2];
                U = (center - point_start).normalized();
                V = (center - point_end).normalized();
                t = M_PI;

                if_reverse = false;

                std::cout << " insert start(2) < end(2) d1 < d2" << std::endl;
            } else {
                center << point_end[0], point_end[1], point_start[2];
                U = (center - point_end).normalized();
                V = (center - point_start).normalized();
                t = M_PI;

                if_reverse = true;

                std::cout << " insert start(2) > end(2) d1 < d2" << std::endl;
            }
        }
    }

    W  = U.cross(V);
    a_ = std::max(d1, d2);
    b_ = std::min(d1, d2);

    std::cout << " point start compare is " << (point_start(2) < point_end(2)) << " t is " << t << std::endl;
    std::cout << " point_start " << point_start << "\n"
              << " point end " << point_end << std::endl;
    R << U, V, W;

    c_          = std::sqrt(std::pow(a_, 2) - std::pow(b_, 2));
    traj_length = 0.25 * calculate_perimeter(a_, b_);

    std::vector<BCs<double>> BCs_;
    BCs<double> BC;
    BC.s_state.pos = 0;
    BC.g_state.pos = traj_length;
    BC.max_vel     = _v_max;
    BC.max_acc     = _a_max;
    BC.max_jerk    = _j_max;
    BC.delta_t     = m_time_interval;
    BCs_.push_back(BC);

    velPlanner  = std::make_shared<vp::tp::VelocityPlannerCompat>(BCs_, algo_);
    auto lambda = velPlanner->getPosVec().front();

    VP_TP_DEBUG(traj_length, lambda.back());
    m_trajectory_size = lambda.size();

    Eigen::AngleAxisd angleAxisTransformation;
    Eigen::Vector3d angleAxis;

    int j     = 0;
    double ll = 1.0 / lambda.size();

    Eigen::Quaterniond qf, qt;

    qf = (if_reverse) ? end_q : start_q;
    qt = (if_reverse) ? start_q : end_q;

    // todo 根据弧长 计算 坐标
    for (auto i : lambda) {
        // 当前运动总长度减去上一时刻运动的总长度(归一化后的)  再乘以1/4椭圆周长
        deltaArcLen = (i - lastArcLen) * traj_length;

        next_ellipse_point = calculate_next_ellipse_point(a_, b_, c_, t, deltaArcLen);

        ellipse_point << next_ellipse_point.first, next_ellipse_point.second,
            0.0;                                               // 局部坐标系下的点
        ellipse_point_wrt_base << R * ellipse_point + center;  // 基坐标系下的点

        // 姿态直接起始位置slerp插值
        angleAxisTransformation = qf.slerp(j * ll, qt);

        angleAxis = angleAxisTransformation.angle() * angleAxisTransformation.axis();

        trajectory.push_back({ellipse_point_wrt_base[0], ellipse_point_wrt_base[1], ellipse_point_wrt_base[2],
                              angleAxis(0), angleAxis(1), angleAxis(2)});

        lastArcLen = i;

        if (i == lambda.back() || i == 0) {
            VP_TP_DEBUG(t);
            VP_TP_DEBUG(i);
        }
        j++;
    }

    // 起点与终点连线的 在xy平面上的投影 短于 起点终点z向之差时，起点到终点的轨迹的参数角度为0～90, 不需要reverse轨迹
    // 当起点与终点连线的 在xy平面上的投影 长于 起点终点z向之差时，起点到终点的轨迹的参数角度为90～0,则需要reverse轨迹
    if (if_reverse) {
        std::reverse(trajectory.begin(), trajectory.end());
    }

    std::cout << "if reverse start " << if_reverse << " " << trajectory[0][0] << " " << trajectory[0][1] << " "
              << trajectory[0][2] << std::endl;
    std::cout << "if reverse start " << if_reverse << " "
              << "end " << trajectory[lambda.size() - 1][0] << " " << trajectory[lambda.size() - 1][1] << " "
              << trajectory[lambda.size() - 1][2] << std::endl;

    return trajectory;

#else
    Eigen::Vector3d point_start = m_start_pose.head(3);
    Eigen::Vector3d point_end   = m_end_pose.head(3);

    Eigen::Vector3d vec = point_start - point_end;

    // 长短半轴、焦距计算
    a_ = vec.head(2).norm();  // 半长轴
    b_ = abs(vec(2));         // 半短轴

    std::cout << " the a b is " << a_ << " " << b_ << std::endl;

    if (a_ <= b_) {
        auto tmp = b_;
        b_       = a_;
        a_       = tmp;
    }

    c_ = std::sqrt(std::pow(a_, 2) - std::pow(b_, 2));

    double traj_length = 0.25 * calculate_perimeter(a_, b_);

    std::cout << " the a b c is " << a_ << " " << b_ << " " << c_ << std::endl;
    std::cout << " traj_length is  " << traj_length << std::endl;

    // 椭圆中心
    Eigen::Vector3d center, U, V, W;
    if (_traj_type == insert_trajectory) {
        center << point_end[0], point_end[1], point_start[2];
        U = (point_start - center).normalized();
        V = (point_end - center).normalized();
    } else {
        center << point_start[0], point_start[1], point_end[2];
        U = (point_end - center).normalized();
        V = (point_start - center).normalized();
    }

    W = U.cross(V);

    Eigen::Matrix3d R;  // 椭圆中心坐标系姿态
    R << U, V, W;

    double t = 0.0, deltaArcLen = 0.0, lastArcLen = 0.0;
    Eigen::Matrix3d R_point;
    Eigen::Vector3d ellipse_point, ellipse_point_wrt_base;
    std::vector<std::vector<double>> trajectory;

    std::pair<double, double> next_ellipse_point;

    std::pair<std::vector<double>, std::vector<double>> p_and_v = TrapezoidalVelocityTrajectory::normalization_factor(
        0, traj_length, 0, 0, _v_max, _a_max, -_a_max, 0.0, m_time_interval);

    std::vector<double> lambda = p_and_v.first;  // 位置的归一化
    std::vector<double> lambda;                  // 位置的归一化

    Eigen::Vector3d angleAxis;
    Eigen::AngleAxisd angleAxisTransformation;

    Eigen::Quaterniond qf = end_q;
    Eigen::Quaterniond qt = start_q;
    Eigen::Quaterniond q;

    m_trajectory_size = lambda.size();

    VP_TP_DEBUG(m_trajectory_size);

    // todo 根据弧长 计算 坐标
    for (auto i : lambda) {
        // 当前运动总长度减去上一时刻运动的总长度(归一化后的)  再乘以1/4椭圆周长
        deltaArcLen = (i - lastArcLen) * traj_length;

        next_ellipse_point = calculate_next_ellipse_point(a_, b_, c_, t, deltaArcLen);

        ellipse_point << next_ellipse_point.first, next_ellipse_point.second,
            0.0;                                               // 局部坐标系下的点
        ellipse_point_wrt_base << R * ellipse_point + center;  // 基坐标系下的点

        // 椭圆上点的跟随坐标系的X轴和Y轴
        double x_dot = -a_ * sin(t);  // x对t求导
        double y_dot = b_ * cos(t);   // y对t求导

        Eigen::Vector3d V_point = W;
        Eigen::Vector3d W_point = R * Eigen::Vector3d({x_dot, y_dot, 0}).normalized();
        Eigen::Vector3d U_point = V_point.cross(W_point);
        R_point << U_point, V_point, W_point;

        // 姿态直接起始位置slerp插值
        angleAxisTransformation = qf.slerp(i, qt);

        angleAxis = angleAxisTransformation.angle() * angleAxisTransformation.axis();

        trajectory.push_back({ellipse_point_wrt_base[0], ellipse_point_wrt_base[1], ellipse_point_wrt_base[2],
                              angleAxis(0), angleAxis(1), angleAxis(2)});

        lastArcLen = i;
    }

    return trajectory;
#endif
}

void EllipseTrajectory::set_start_pose(std::array<double, 6> _start_pose) {
    m_start_pose = Eigen::Map<Eigen::Matrix<double, 6, 1>>(_start_pose.data());
}

void EllipseTrajectory::set_end_pose(std::array<double, 6> _end_pose) {
    m_end_pose = Eigen::Map<Eigen::Matrix<double, 6, 1>>(_end_pose.data());
}

void EllipseTrajectory::set_timeinterval(double _time_interval) {
    m_time_interval = _time_interval;
}

double EllipseTrajectory::get_traj_time() {
    return m_trajectory_size * m_time_interval;
}