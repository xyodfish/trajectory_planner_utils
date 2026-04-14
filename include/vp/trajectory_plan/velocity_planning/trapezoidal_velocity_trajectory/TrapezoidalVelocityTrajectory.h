//
// Created by hengjinli on 23-7-25.
//

#ifndef TRAPEZOIDAL_VELOCITY_TRAJECTORY_H
#define TRAPEZOIDAL_VELOCITY_TRAJECTORY_H

#include <vector>

class TrapezoidalVelocityTrajectory {
   public:
    TrapezoidalVelocityTrajectory();

    ~TrapezoidalVelocityTrajectory();

   public:
    /**
     * 梯形速度规划
     * @param _p0 初始点
     * @param _p1 终止点
     * @param _v0 初速度
     * @param _v1 终止速度
     * @param _v_max 最大速度
     * @param _aa 加速度
     * @param _ad 减速度（带符号）
     * @param _t0 初始时间
     * @param _time_interval 采样时间
     * @return 轨迹点（first），轨迹点速度（second）
     */
    static std::pair<std::vector<double>, std::vector<double>> trapezoidal_velocity_trajectory(
        double _p0, double _p1, double _v0, double _v1, double _v_max, double _aa, double _ad, double _t0 = 0.0,
        double _time_interval = 0.001);

    /**
     * 梯形速度规划归一化因子
     * @param _p0 初始点
     * @param _p1 终止点
     * @param _v0 初速度
     * @param _v1 终止速度
     * @param _v_max 最大速度
     * @param _aa 加速度
     * @param _ad 减速度（带符号）
     * @param _t0 初始时间
     * @param _time_interval 采样时间
     * @return 轨迹点
     */
    static std::pair<std::vector<double>, std::vector<double>> normalization_factor(double _p0, double _p1, double _v0,
                                                                                    double _v1, double _v_max,
                                                                                    double _aa, double _ad,
                                                                                    double _t0 = 0.0,
                                                                                    double _time_interval = 0.001);

   private:
};

#endif  // TRAPEZOIDAL_VELOCITY_TRAJECTORY_H
