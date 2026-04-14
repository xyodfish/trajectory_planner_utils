/**
 * @file traj.h
 * @author original.author@example.com
 * @brief 轨迹相关功能
 * @version 0.1
 * @date 2023-10-23
 *
 * @copyright Copyright (c) 2023 Original Source
 */

#ifndef TRAJ_IO_H_
#define TRAJ_IO_H_

#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <vector>

#include "traj_io_std.h"
namespace vp::tp {
    /**
     * @brief 读取路径
     * @param path 路径
     * @param file_name 路径文件
     * @param dim 路径维度
     */
    void readPath(std::vector<std::vector<double>>& path, const std::string& file_name);

    enum TrajIndexEnum { time_index_ = 0, pose_index_ = 1, vel_index_ = 7, acc_index_ = 13 };

    /**
     * @brief 轨迹点结构体
     */
    struct TrajPoint {
        /// @brief 轨迹点时间戳
        double time = 0;
        /// @brief 轨迹点位置
        Eigen::VectorXd pos;
        /// @brief 轨迹点速度
        Eigen::VectorXd vel;
    };

    /**
     * @brief 笛卡尔空间轨迹规划
     * @param path 路径数组
     * @param config_file 配置文件
     * @param traj 轨迹
     */
    bool getTrajL(const std::vector<std::vector<double>>& path, const std::string& config_file,
                  std::vector<TrajPoint>& traj, double time_step);

    /**
     * @brief 读取笛卡尔空间轨迹
     * @param traj 轨迹（TrajPoint数组）
     * @param file_name 轨迹文件
     */
    void readTrajL(std::vector<TrajPoint>& traj, const std::string& file_name);

    /**
     * @brief 写入轨迹
     * @param traj 轨迹（TrajPoint数组）
     * @param file 轨迹文件
     * @param separator 分隔符
     */
    void writeTrajL(const std::vector<TrajPoint>& traj, std::string file, const std::string& separator = "    ");

    /**
     * @brief 计算圆弧圆心（给定圆弧起点、终点位置、轴线矢量、半径）
     * @param start_pos 圆弧起点位置
     * @param end_pos 圆弧终点位置
     * @param axis 旋转轴线矢量（绕此矢量正转）
     * @param radius 圆弧半径
     * @param smaller_than_pi 圆弧对应圆心角角度是否小于PI
     * @return Eigen::Vector3d 圆弧圆心位置
     */
    Eigen::Vector3d calArcCenter(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos, Eigen::Vector3d axis,
                                 double radius, bool smaller_than_pi = true);

    /**
     * @brief 计算圆弧轴线矢量（给定圆弧起点、终点位置、圆心位置）
     * @param start_pos 圆弧起点位置
     * @param end_pos 圆弧终点位置
     * @param center_pos 圆心位置
     * @param positive 是否绕轴线矢量正转
     * @return Eigen::Vector3d 圆弧轴线矢量
     */
    Eigen::Vector3d calArcAxis(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos,
                               const Eigen::Vector3d& center_pos, bool positive,
                               const Eigen::Vector3d* mid_pos_ptr = nullptr);

    /**
     * @brief 计算圆弧圆心，圆弧轴线矢量（给定圆弧起点、终点位置、估计圆心位置）
     * @param start_pos 圆弧起点位置
     * @param end_pos 圆弧终点位置
     * @param center_pos_hint 估计圆心位置
     * @param positive 是否绕轴线矢量正转
     * @return std::pair<Eigen::Vector3d, Eigen::Vector3d> {圆弧圆心位置，圆弧轴线矢量}
     */
    std::pair<Eigen::Vector3d, Eigen::Vector3d> calArcCenterAxis(const Eigen::Vector3d& start_pos,
                                                                 const Eigen::Vector3d& end_pos,
                                                                 const Eigen::Vector3d center_pos_hint, bool positive);

    Eigen::Vector3d calTrajAxisX(const std::vector<std::vector<double>>& traj, int index, int window_size = 1);
    Eigen::Vector3d calTransTrajAxisX(const std::vector<std::vector<double>>& traj, int index, int window_size = 1);
}  // namespace vp::tp

#endif