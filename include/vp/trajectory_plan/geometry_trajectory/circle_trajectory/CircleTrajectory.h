/**
 * @file CircleTrajectory.h
 * @brief 圆弧轨迹规划
 * @version 0.1
 * @date 2023-11-16
 *
 * @copyright Copyright (c) 2023 Original Source
 */

#ifndef _CIRCLE_TRAJECTORY_H
#define _CIRCLE_TRAJECTORY_H

#include <Eigen/Dense>
#include <vector>

namespace vp::tp {
    /**
   * @brief 由位置、姿态（四元数），获取Tcp位姿
   * @param pos 位置
   * @param q 姿态（四元数）
   * @return std::vector<double> Tcp位姿
   */
    std::vector<double> getTcpPoseFromPositionQuaternion(Eigen::Vector3d pos, Eigen::Quaterniond q);

    class CircleTrajectory {
       public:
        /// @brief 规划类型
        enum PlanType { START_CENTER_AXIS_LINEAR = 0, START_CENTER_AXIS_ANGULAR, START_CENTER_END_LINEAR, UNKNOWN };

        /**
     * @brief Construct a new Circle Trajectory object
     * 构造时给定起点位姿、圆心位置，规划时给定旋转轴方向矢量、旋转角度，做线速度或角速度规划
     * @param _start_pose 起始位姿
     * @param _center_pos 圆心位置
     * @param is_linear 是否线速度规划
     */
        explicit CircleTrajectory(std::vector<double> _start_pose, std::vector<double> _center_pos,
                                  bool is_linear = true);

        /**
     * @brief Construct a new Circle Trajectory object
     * 构造时给定起点位姿、圆心位置、终点位姿，规划时给定绕轴正/反转标志，做线速度规划，各轨迹点姿态由起点、终点姿态插值得到
     * @param _start_pose 起始位姿
     * @param _center_pos 中点位置
     * @param _end_pose
     */
        explicit CircleTrajectory(std::vector<double> _start_pose, std::vector<double> _end_pose,
                                  std::vector<double> _center_pose);

        ~CircleTrajectory();

        /**
     * @brief Set the timeinterval 设置轨迹点时间间隔
     * @param _time_interval 轨迹点时间间隔
     */
        void set_timeinterval(double _time_interval);

        /**
     * @brief
     * 给定起点位姿、圆心位置，旋转轴方向矢量和旋转角度的圆弧，可做线/角速度规划
     * @param _v_max 速度最大值
     * @param _a_max 加速度最大值
     * @param axis 旋转轴方向矢量（绕轴正转）
     * @param theta 角度（rad）
     * @return std::vector<std::vector<double>> 轨迹
     */
        std::vector<std::vector<double>> cal_trapezoidal_vel_trajectory(double _v_max, double _a_max,
                                                                        Eigen::Vector3d axis, double theta);

        /**
     * @brief 根据起点、圆心、终点确定圆弧，且保证姿态插值，仅做线速度规划
     * @param _v_max 速度最大值
     * @param _a_max 加速度最大值
     * @param positive 是否绕轴正转
     * @return std::vector<std::vector<double>> 轨迹
     */
        std::vector<std::vector<double>> cal_trapezoidal_vel_trajectory(double _v_max, double _a_max,
                                                                        bool positive = true);

        void cal_trapezoidal_vel_trajectory_help_function(std::vector<std::vector<double>>& trajectory,
                                                          double traj_length, double _v_max, double _a_max);

       private:
        /// @brief 起点位姿
        Eigen::Matrix<double, 6, 1> m_start_pose;
        /// @brief 终点位姿
        Eigen::Matrix<double, 6, 1> m_end_pose;

        /// @brief 起点姿态，四元数表示
        Eigen::Quaterniond m_start_q;
        /// @brief 终点姿态，四元数表示
        Eigen::Quaterniond m_end_q;

        /// @brief 起点位置
        Eigen::Vector3d m_start_pos;
        /// @brief 终点位置
        Eigen::Vector3d m_end_pos;
        /// @brief 圆心位置
        Eigen::Vector3d m_center_pos;

        /// @brief 旋转轴方向矢量
        Eigen::Vector3d m_axis;

        /// @brief 时间间隔
        double m_time_interval = 0.01;

        /// @brief 圆弧半径
        double m_radius = 0.0;

        /// @brief 旋转角度
        double m_theta;

        PlanType m_plan_type = PlanType::UNKNOWN;
    };
}  // namespace vp::tp
#endif
