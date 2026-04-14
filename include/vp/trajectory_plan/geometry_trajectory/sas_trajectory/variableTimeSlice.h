#ifndef VP_TP_VARIABLE_TIME_SLICE_H
#define VP_TP_VARIABLE_TIME_SLICE_H

#include <cmath>
#include <cstdint>
#include <vector>
#ifdef VP_TP_ENABLE_PLOTTING
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

class VariableTimeSlice {
   private:
    double t_simu_;
    double old_t_simu_;
    double old_ratio_;
    double cur_ratio_;
    double var_ratio_;
    double exp_ratio_;
    double det_rt_;
    double acc_sum_t_;

    uint8_t start_flag_;
    uint8_t v_stage_;
    std::vector<double> simu_time_slice_;
    std::pair<double, double> time_span_;

    std::vector<double> new_st_vec;

    std::vector<double> cur_ratio_vec;

   public:
    explicit VariableTimeSlice() = default;
    VariableTimeSlice(double var_ratio, double exp_ratio, double det_rt, double t_simu, double time_ratio,
                      const std::pair<double, double>& time_span) {

        setTimeSlice(var_ratio, exp_ratio, det_rt, t_simu, time_ratio, time_span);
    }

    void setTimeSlice(double var_ratio, double exp_ratio, double det_rt, double t_simu, double time_ratio,
                      const std::pair<double, double>& time_span) {

        t_simu_     = t_simu;
        old_t_simu_ = t_simu;
        det_rt_     = det_rt;
        old_ratio_ = cur_ratio_ = time_ratio;

        acc_sum_t_ = 0;

        var_ratio_       = var_ratio;
        exp_ratio_       = exp_ratio;
        start_flag_      = 0;
        v_stage_         = 0;
        simu_time_slice_ = {};

        time_span_ = time_span;

        new_st_vec.clear();
        cur_ratio_vec.clear();
    }

    double getRealTimePeriod(double t_simu) {

        if (start_flag_ == 0) {
            start_flag_ = 1;
        }

        if (start_flag_) {
            switch (v_stage_) {
                case 0:
                    initializeVariableStage(t_simu);
                    break;
                case 1:
                    firstVariableStage(t_simu);
                    break;
                case 2:
                    secondVariableStage(t_simu);
                    break;
                case 3:
                    thirdVariableStage(t_simu);
                    break;
                default:
                    break;
            }

            updateData();
        }

        return t_simu_;
    }

    void plot_new_rt() {
#ifdef VP_TP_ENABLE_PLOTTING
        plt::plot(cur_ratio_vec);
        plt::show();
#endif
    }

    double getTimeRatio() { return cur_ratio_; }

   private:
    void calculateTrapTime() {
        double loop_ratio     = 1.0;
        double loop_var_ratio = var_ratio_;

        while (true) {

            while (true) {
                if (loop_ratio < exp_ratio_ - 1e-3) {
                    break;
                }

                acc_sum_t_ += loop_ratio * det_rt_;
                loop_ratio = loop_ratio + loop_var_ratio;
            }

            if ((time_span_.second - time_span_.first) > 2 * acc_sum_t_ + det_rt_) {
                var_ratio_ = loop_var_ratio;
                break;
            } else {
                acc_sum_t_     = 0;
                loop_ratio     = 1.0;
                loop_var_ratio = 1.1 * loop_var_ratio;
            }
        }
    }

    void initializeVariableStage(double t_simu) {

        v_stage_ = 1;

        calculateTrapTime();

        double fixed_simu_t1_ = time_span_.first + acc_sum_t_;
        double fixed_simu_t2_ = time_span_.second - acc_sum_t_;

        simu_time_slice_.push_back(time_span_.first);
        simu_time_slice_.push_back(fixed_simu_t1_);
        simu_time_slice_.push_back(fixed_simu_t2_);
        simu_time_slice_.push_back(time_span_.second);

        t_simu_ = t_simu_ + det_rt_;

        cur_ratio_vec.push_back(1.0);
    }

    void firstVariableStage(double t_simu) {

        cur_ratio_ = cur_ratio_ + var_ratio_;

        if (cur_ratio_ <= exp_ratio_) {
            cur_ratio_ = exp_ratio_;
        }

        t_simu_ = t_simu_ + det_rt_ * cur_ratio_;

        if (t_simu >= simu_time_slice_[v_stage_]) {
            v_stage_ = 2;
        }

        new_st_vec.push_back(t_simu_ - old_t_simu_);
        cur_ratio_vec.push_back(cur_ratio_);
    }

    void secondVariableStage(double t_simu) {

        t_simu_ = t_simu_ + det_rt_ * cur_ratio_;
        // dbg(cur_ratio_);

        if (t_simu > simu_time_slice_[v_stage_]) {
            v_stage_ = 3;
        }
        new_st_vec.push_back(t_simu_ - old_t_simu_);
        cur_ratio_vec.push_back(cur_ratio_);
    }

    void thirdVariableStage(double t_simu) {

        cur_ratio_ = old_ratio_ - var_ratio_;

        if (cur_ratio_ >= 1.0) {
            cur_ratio_ = 1.0;
        }

        t_simu_ = t_simu_ + det_rt_ * cur_ratio_;

        new_st_vec.push_back(t_simu_ - old_t_simu_);
        cur_ratio_vec.push_back(cur_ratio_);

        if (std::fabs(t_simu - simu_time_slice_[v_stage_]) < 0.001) {
            start_flag_ = 0;
        }
    }

    void updateData() {
        old_ratio_  = cur_ratio_;
        old_t_simu_ = t_simu_;
    }
};

#endif
