//
// Created by hengjinli on 23-7-25.
//

#include "TrapezoidalVelocityTrajectory.h"
#include <cmath>
#include <iostream>

TrapezoidalVelocityTrajectory::TrapezoidalVelocityTrajectory() {}

TrapezoidalVelocityTrajectory::~TrapezoidalVelocityTrajectory() {}

std::pair<std::vector<double>, std::vector<double>> TrapezoidalVelocityTrajectory::trapezoidal_velocity_trajectory(
    double _p0, double _p1, double _v0, double _v1, double _v_max, double _aa, double _ad, double _t0,
    double _time_interval) {
    // ##########################梯形速度规划初始化##########################--------
    double h = std::abs(_p1 - _p0);
    double vf = std::sqrt((2.0 * _aa * _ad * h - _aa * std::pow(_v1, 2) + _ad * std::pow(_v0, 2)) / (_ad - _aa));

    double vv;  // 匀速段速度
    if (vf < _v_max) {
        vv = vf;  // 不能达到最大速度
    } else {
        vv = _v_max;  // 可以达到最大速度
    }

    // 加速段
    double T_a = (vv - _v0) / _aa;                          // 加速段的时间
    double L_a = _v0 * T_a + 0.5 * _aa * std::pow(T_a, 2);  // 加速段的位移

    // 匀速段
    double T_v =
        (h - (std::pow(vv, 2) - std::pow(_v0, 2)) / (2.0 * _aa) - (std::pow(_v1, 2) - std::pow(vv, 2)) / (2.0 * _ad)) /
        vv;                 // 匀速段的时间
    double L_v = vv * T_v;  // 匀速段的位移

    // 减速段
    double T_d = (std::abs(_v1) - vv) / _ad;               // 减速段的时间
    double L_d = vv * T_d + 0.5 * _ad * std::pow(T_d, 2);  // 减速段的位移

    double T = 0.0 + T_a + T_v + T_d;  // 总时间
    // ##########################梯形速度规划初始化##########################--------

    double delta_t = _time_interval;
    size_t step = T / delta_t;

    std::vector<double> q;   // 轨迹点
    std::vector<double> dq;  // 轨迹点速度

    for (int i = 0; i < step; ++i) {
        double t_current = i * delta_t - _t0;  // 当前时间
        double q_temp;
        double dq_temp;

        if (t_current >= 0 && t_current < T_a) {  // 加速
            q_temp = _p0 + _v0 * t_current + 0.5 * _aa * std::pow(t_current, 2);
            dq_temp = _v0 + _aa * t_current;
        } else if (t_current >= T_a && t_current < (T_a + T_v)) {  // 匀速
            q_temp = _p0 + L_a + vv * (t_current - T_a);
            dq_temp = vv;
        } else {  // 减速
            q_temp = _p0 + L_a + L_v + vv * (t_current - T_a - T_v) + 0.5 * _ad * std::pow(t_current - T_a - T_v, 2);
            dq_temp = vv + _ad * t_current;
        }

        q.push_back(q_temp);
        dq.push_back(dq_temp);
    }

    return std::make_pair(q, dq);
}

std::pair<std::vector<double>, std::vector<double>> TrapezoidalVelocityTrajectory::normalization_factor(
    double _p0, double _p1, double _v0, double _v1, double _v_max, double _aa, double _ad, double _t0,
    double _time_interval) {
    // spdlog::info(
    //     "_p0: {}, _p1: {}, _v0: {}, _v1: {}, _vmax: {}, _aa: {}, _ad: {}, _dt: {}", _p0, _p1, _v0, _v1, _v_max, _aa,
    //     _ad, _time_interval);

    std::cout << " p0 " << _p0 << " p1 " << _p1 << " v0 " << _v0 << " v1 " << _v1 << " vmax " << _v_max << " aa " << _aa
              << "_ad " << _ad << std::endl;

    // ##########################梯形速度规划初始化##########################
    double h = std::abs(_p1 - _p0);
    double vf = std::sqrt((2.0 * _aa * _ad * h - _aa * std::pow(_v1, 2) + _ad * std::pow(_v0, 2)) / (_ad - _aa));

    double vv;  // 匀速段速度
    if (vf < _v_max) {
        vv = vf;  // 不能达到最大速度
    } else {
        vv = _v_max;  // 可以达到最大速度
    }

    // std::cout << " the set vmax is " << _v_max << "\n"
    //           << " the uniform v is " << vv << "\n";

    // 加速段
    double T_a = std::fabs((vv - _v0) / _aa);               // 加速段的时间
    double L_a = _v0 * T_a + 0.5 * _aa * std::pow(T_a, 2);  // 加速段的位移

    // 匀速段
    double T_v = std::fabs(
        (h - (std::pow(vv, 2) - std::pow(_v0, 2)) / (2.0 * _aa) - (std::pow(_v1, 2) - std::pow(vv, 2)) / (2.0 * _ad)) /
        vv);                // 匀速段的时间
    double L_v = vv * T_v;  // 匀速段的位移

    // 减速段
    double T_d = std::fabs((_v1 - vv) / _ad);              // 减速段的时间
    double L_d = vv * T_d + 0.5 * _ad * std::pow(T_d, 2);  // 减速段的位移

    double T = 0.0 + T_a + T_v + T_d;  // 总时间
    // ##########################梯形速度规划初始化##########################

    // ##########################归一化因子初始化##########################
    double S1_ = std::abs(L_a / h);
    double S2_ = std::abs(L_d / h);

    double T1_ = T_a / T;  // 加速段
    double T3_ = T_d / T;  // 减速段
    double T2_ = 1 - T3_;  // 匀速段

    double acc1_;
    if (T1_ != 0)
        acc1_ = 2 * S1_ / std::pow(T1_, 2);  // 加速段
    else
        acc1_ = 0;

    double acc2_;
    if (T2_ != 0)
        acc2_ = 2 * S2_ / std::pow(T3_, 2);  // 减速段
    else
        acc2_ = 0;

    // std::cout << " the increase acc is " << acc1_ << "\n"
    //           << " the decreasd acc v is " << acc2_ << "\n";

    // std::cout << " the acc t is " << T_a << "\n"
    //           << " the uniform t is " << T_v << "\n"
    //           << " the decrease t is " << T_d << "\n"
    //           << " the total t is " << T << "\n";
    // ##########################归一化因子初始化##########################

    double delta_t = _time_interval;
    size_t step = T / delta_t;

    std::vector<double> lambda;    // 归一化因子
    std::vector<double> lambda_v;  // 归一化因子_速度
    for (int i = 0; i < step; ++i) {
        double t_current = i * delta_t - _t0;   // 当前时间
        double t_normalized = i * delta_t / T;  // 归一化时间

        double p;
        double v;
        double a;
        if (t_current >= 0 && t_current < T_a) {  // 加速
            p = 0.5 * acc1_ * std::pow(t_normalized, 2);
            v = acc1_ * t_normalized;
        } else if (t_current >= T_a && t_current < (T_a + T_v)) {  // 匀速
            if (T1_ == 0) {
                p = acc2_ * T3_ * t_normalized;
                v = acc2_ * T3_;
            } else {
                p = 0.5 * acc1_ * std::pow(T1_, 2) + acc1_ * T1_ * (t_normalized - T1_);
                v = acc1_ * T1_;
            }
        } else {  // 减速
            if (T1_ == 0) {
                p = acc2_ * T3_ * (T2_ - T1_) + acc2_ * T3_ * (t_normalized - T2_) -
                    .5 * acc2_ * std::pow(t_normalized - T2_, 2);
                v = acc2_ * T3_ - acc2_ * t_normalized;
            } else {
                p = .5 * acc1_ * std::pow(T1_, 2) + acc1_ * T1_ * (T2_ - T1_) + acc1_ * T1_ * (t_normalized - T2_) -
                    .5 * acc2_ * std::pow(t_normalized - T2_, 2);
                v = acc1_ * T1_ - acc2_ * (t_normalized - T2_);
            }
        }

        lambda.push_back(p);
        lambda_v.push_back(v);
    }

    return std::make_pair(lambda, lambda_v);
}
