/**
 * @file traj_io_std.h
 * @author original.author@example.com
 * @brief 
 * @version 0.1
 * @date 2024-12-17
 * 
 * @copyright Copyright (c) 2024 Original Source
 */

#ifndef TRAJ_IO_STD_H_
#define TRAJ_IO_STD_H_

#include <string>
#include <vector>

namespace vp::tp {
    /**
     * @brief 读取笛卡尔空间轨迹
     * @param traj 轨迹（TrajPoint数组）
     * @param file_name 轨迹文件
     * @param dim 轨迹维度
     * @param differential_order 微分阶次
     */
    void readTrajLSTD(std::vector<std::vector<double>>& traj, std::string file_name, int dim, int differential_order);

    /**
     * @brief 读取笛卡尔空间轨迹
     * @param traj 轨迹（TrajPoint数组）
     * @param file_name 轨迹文件
     * @param dim 轨迹维度
     * @param differential_order 微分阶次
     */
    void readTrajLTransSTD(std::vector<std::vector<double>>& traj, std::string file_name, int dim,
                           int differential_order);

    /**
     * @brief 写入轨迹
     * @param traj 轨迹（TrajPoint数组）
     * @param file 轨迹文件
     * @param separator 分隔符
     */
    void writeTrajLSTD(const std::vector<std::vector<double>>& traj, const std::string& file,
                       const std::string& separator = "    ");

    /**
     * @brief 写入轨迹
     * @param traj 轨迹（TrajPoint数组）
     * @param file 轨迹文件
     * @param separator 分隔符
     */
    void writeTrajLTransSTD(const std::vector<std::vector<double>>& traj, const std::string& file,
                            const std::string& separator = "    ");

    /**
     * @brief 轨迹转置
     * @param src 源轨迹
     * @param dst 目标轨迹
     */
    void trajTrans(const std::vector<std::vector<double>>& src, std::vector<std::vector<double>>& dst);
}  // namespace vp::tp
#endif