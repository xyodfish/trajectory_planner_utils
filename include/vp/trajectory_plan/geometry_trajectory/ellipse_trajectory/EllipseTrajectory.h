//
// Created by hengjinli on 23-8-3.
//

#ifndef ELLIPSE_TRAJECTORY_H
#define ELLIPSE_TRAJECTORY_H

#include <Eigen/Dense>
#include <array>
#include <memory>
#include <vector>
#include "VelocityPlannerCompat.h"

class EllipseTrajectory {
   public:
    explicit EllipseTrajectory(std::array<double, 6> _start_pose, std::array<double, 6> _end_pose);
    explicit EllipseTrajectory(std::vector<double> _start_pose, std::vector<double> _end_pose);

    ~EllipseTrajectory();

    typedef enum {
        insert_trajectory = 0,
        buckle_trajectory = 1

    } EllipseTrajType;

   public:
    /**
     * 设置起始和终止位姿
     * @param _start_pose
     * @param _end_pose
     */
    void set_start_end_pose(std::array<double, 6> _start_pose, std::array<double, 6> _end_pose);

    /**
     * 设置起始位姿
     * @param _start_pose
     */
    void set_start_pose(std::array<double, 6> _start_pose);

    /**
     * 设置终止位姿
     * @param _end_pose
     */
    void set_end_pose(std::array<double, 6> _end_pose);

    void set_timeinterval(double _time_interval);

    /**
     * 计算轨迹上点的位姿
     * @param _v_max 最大速度
     * @param _a_max 加减速度
     * @param _traj_type 轨迹类型
     * @return 位姿序列
     */
    std::vector<std::array<double, 6>> cal_trapezoidal_rad_vel_trajectory(
        double _v_max = 0.1, double _a_max = 0.1, EllipseTrajType _traj_type = buckle_trajectory);

    std::vector<std::vector<double>> cal_trapezoidal_linear_vel_trajectory(double _v_max, double _a_max, double _j_max,
                                                                           EllipseTrajType _traj_type);

    double get_traj_time();

    void set_velocity_planner(const std::vector<vp::tp::BCs<double>>& BCs, const std::string& algo);
    void set_planner_name(const std::string& algo);
    void set_boundary_conditions(const std::vector<vp::tp::BCs<double>>& BCs);

   private:
    Eigen::Matrix<double, 6, 1> m_start_pose;
    Eigen::Matrix<double, 6, 1> m_end_pose;
    Eigen::Quaterniond start_q, end_q;
    double m_time_interval   = 0.01;
    double m_trajectory_size = 0.0;

    // 半长轴、半短轴、焦距
    double a_, b_, c_;

    double traj_length;

    /**
     * 计算椭圆的周长
     * @param a 椭圆长轴长度
     * @param b 椭圆短轴长度
     * @return 椭圆周长
     */
    double calculate_perimeter(double a, double b);

    /**
     * 计算椭圆某一点的曲率半径
     * @param a 椭圆长轴长度
     * @param b 椭圆短轴长度
     * @param t 椭圆该点参数t
     * @return 该点的曲率半径
     */
    double calculate_curvature_radius(double a, double b, double t);

    /**
     * 计算椭圆某一点的曲率半径
     * @param a 椭圆长轴长度
     * @param b 椭圆短轴长度
     * @param x 椭圆该点坐标x
     *
     * @return 该点的曲率半径
     */
    double calculate_curvature_radius(double a, double b, double x, double y);

    /**
     * 计算椭圆某一点的曲率半径
     * @param a 椭圆长轴长度
     * @param b 椭圆短轴长度
     * @param c 焦距
     * @param theta 椭圆该点参数t
     * @return 该点的曲率半径
     */
    std::pair<double, double> calculate_curve_circle_center(double a, double b, double c, double t);

    /**
     * 计算当前圆心角度对应位置 延伸一段弧长后椭圆的上的某个点的坐标
     * @param theta 当前圆心角
     * @param arcLen 延伸的弧长
     * @return 延伸后坐标 first:x, second:y
     */
    std::pair<double, double> calculate_next_ellipse_point(double a, double b, double c, double& theta, double arcLen);

    std::pair<double, double> calculate_this_ellipse_point(double a, double b, double theta);

    std::pair<double, double> calculate_next_ellipse_xy(double deltaTheta, std::pair<double, double> point,
                                                        std::pair<double, double> center);

    std::shared_ptr<vp::tp::VelocityPlannerCompat> velPlanner;

    std::vector<vp::tp::BCs<double>> BCs_;
    std::string algo_;
};

#endif  // ELLIPSE_TRAJECTORY_H
