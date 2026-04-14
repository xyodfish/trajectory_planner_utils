#include "sasArcTimeNonUniformTraj.h"
#include "common_functions.h"
#include "traj.h"

using namespace vp::tp::SAS;

namespace vp::tp {

    MultiPointsTrajectory::SASArcTimeNonUniformTraj::SASArcTimeNonUniformTraj(
        const std::shared_ptr<SAS::PiecewiseSegments>& ps_sptr, const std::shared_ptr<VelocityPlannerCompat>& vel_planner,
        MultiPointsTrajectory* mpt_ptr)
        : ps_sptr_(ps_sptr), vel_planner_(vel_planner), mpt_ptr_(mpt_ptr) {
        initialization();
    }

    void MultiPointsTrajectory::SASArcTimeNonUniformTraj::initialization() {
        vts_     = std::make_shared<VariableTimeSlice>();
        data_    = vel_planner_->getTrajs(true);
        dis_vec_ = vel_planner_->getPosVec().front();
        dt_      = data_[1][time_index_] - data_[0][time_index_];
        end_t_   = data_.back()[time_index_];
        num_     = data_.size();
    }

    void MultiPointsTrajectory::SASArcTimeNonUniformTraj::updateSimuTime(bool in_arc) {

        if (!in_arc) {
            t_simu_ += dt_ * time_ratio_ * tsc_;
        } else {
            t_simu_ = vts_->getRealTimePeriod(t_simu_);
        }
    }

    void MultiPointsTrajectory::SASArcTimeNonUniformTraj::updateTimeRatio(bool in_arc) {

        ///
        if (in_arc) {
            time_ratio_ = vts_->getTimeRatio();
        } else if (time_ratio_ < DEF_RATIO) {
            time_ratio_ -= VAR_RATIO;
            time_ratio_ = std::min(time_ratio_, DEF_RATIO);
        }
    }

    void MultiPointsTrajectory::SASArcTimeNonUniformTraj::updataTrajectoryTimeSlice(double t) {

        size_t id = t / dt_;
        new_si    = ps_sptr_->_getSegInfo(dis_vec_[id]);

        if (new_si.second == SegType::STRAIGHT && old_si.second == SegType::ARC) {
            if (in_arc_) {
                in_arc_ = false;
                // vts_->plot_new_rt();
            }
        } else {

            if (new_si.second == SegType::ARC && old_si.second == SegType::STRAIGHT) {
                in_arc_ = true;

                auto l1 = ps_sptr_->_getSegLengthPeriod(new_si.first);
                auto l2 = ps_sptr_->_getSegLengthPeriod(new_si.first + 1);

                auto end_time_id = findVectorLowerIndex(dis_vec_, l2.second);
                auto end_time    = data_[end_time_id][time_index_];

                vts_->setTimeSlice(VAR_RATIO, EXP_RATIO, dt_, t, time_ratio_, std::pair<double, double>{t, end_time});
            }
        }

        old_si = new_si;
    }

    void MultiPointsTrajectory::SASArcTimeNonUniformTraj::calTrajPose(size_t id, double ratio) {

        auto iter = getInterpolation(data_[id][pose_index_], data_[id + 1][pose_index_], ratio);
        mpt_ptr_->appendTrajData(iter, val_, t_real_, dt_, cur_id_);

        t_real_ += dt_;
        cur_id_++;
    }

    std::vector<std::vector<double>> MultiPointsTrajectory::SASArcTimeNonUniformTraj::getVarySpeedTraj() {

        while (true) {
            if (t_simu_ >= end_t_) {
                break;
            }

            if (cur_id_ == 1 && tsc_ == 0) {
                tsc_ = 1;
            }

            updataTrajectoryTimeSlice(t_simu_);
            updateSimuTime(in_arc_);
            updateTimeRatio(in_arc_);

            auto res = calculateRatio(t_simu_, dt_);
            if (res.first >= num_ - 1) {
                break;
            }

            calTrajPose(res.first, res.second);
        }

        return val_;
    }
}  // namespace vp::tp
