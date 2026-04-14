//
// Created by hengjinli on 23-7-25.
//

#ifndef STRAIGHT_ARC_TRANSITION_H
#define STRAIGHT_ARC_TRANSITION_H

#include <Eigen/Dense>
#include <array>
#include <vector>

class StraightArcTransition {
   public:
    StraightArcTransition();

    ~StraightArcTransition();

   public:
    /**
   * 计算圆弧轨迹上的各点姿态
   * @param _point0 起始点
   * @param _point1 过渡点
   * @param _point2 终止点
   * @param r 过度圆弧
   * @return 轨迹上的各点姿态
   */
    std::vector<std::array<double, 6>> calculate_transition_arc(std::array<double, 3> _point0,
                                                                std::array<double, 3> _point1,
                                                                std::array<double, 3> _point2, double r = 0.05);

    /**
   * 设置时间间隔
   * @param _time
   */
    void set_time_interval(double _time);

    /**
   * 设置最大速度
   * @param _v_max
   */
    void set_v_max(double _v_max);

    /**
   * 设置加减速度
   * @param _a_max
   */
    void set_a_max(double _a_max);

   private:
    /**
   * 给定过渡圆弧半径，计算过渡点、过渡圆弧
   * @param _point0 起始点
   * @param _point1 过渡点
   * @param _point2 终止点
   * @param _r 过渡圆弧半径
   * @return 过渡点1，过渡点2，圆心
   */
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> segmented_path_transfer(
        const Eigen::Vector3d& _point0, const Eigen::Vector3d& _point1, const Eigen::Vector3d& _point2, double _r);

    /**
   * 计算直线圆弧过渡点的初始位姿
   * @param _point0 起始点
   * @param _point1 终止点
   * @param _center 圆心
   * @return
   */
    Eigen::Matrix3d calculate_initial_pose(const Eigen::Vector3d& _point0, const Eigen::Vector3d& _point1,
                                           const Eigen::Vector3d& _center);

    /**
   * 计算圆弧上每一点的姿态
   * @param _axis 旋转轴
   * @param _initial_radius_vector 初始要旋转的向量
   * @param _center 圆心
   * @param _theta_current 旋转角度
   * @return
   */
    // TODO
    std::pair<Eigen::Vector3d, Eigen::Vector3d> calculate_circular_arc_pose(const Eigen::Vector3d& _axis,
                                                                            Eigen::Vector3d _initial_radius_vector,
                                                                            Eigen::Vector3d _center,
                                                                            double _theta_current);

    /**
   * 计算平面的法向矢量
   * @param _point0 起始点
   * @param _point1 终止点
   * @param _center 圆心
   * @return 法向矢量
   */
    Eigen::Vector3d calculate_plane_normal_vector(const Eigen::Vector3d& _point0, const Eigen::Vector3d& _point1,
                                                  const Eigen::Vector3d& _center);

    /**
   * 计算圆弧角度
   * @param _point0 起点
   * @param _point1 终点
   * @param _center 圆心
   * @return 角度
   */
    double calculate_angle(const Eigen::Vector3d& _point0, const Eigen::Vector3d& _point1,
                           const Eigen::Vector3d& _center) const;

   private:
    double m_vv;                            // 匀速段速度
    double m_a;                             // 加减速度
    double m_time_interval;                 // 采样时间
    Eigen::Matrix3d m_circle_initial_pose;  // 直线圆弧过渡点的初始位姿
};

#endif  // STRAIGHT_ARC_TRANSITION_H