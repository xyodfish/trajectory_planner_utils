#ifndef __SAS_VARY_SPEED_TRAJECTORY_IMPL_H__
#define __SAS_VARY_SPEED_TRAJECTORY_IMPL_H__

#include "multiPointsTrajectory.h"
#include "variableTimeSlice.h"

namespace vp::tp {
    class MultiPointsTrajectory::SASArcTimeNonUniformTraj {

       public:
        explicit SASArcTimeNonUniformTraj() = default;
        explicit SASArcTimeNonUniformTraj(const std::shared_ptr<SAS::PiecewiseSegments>& ps_sptr,
                                          const std::shared_ptr<VelocityPlannerCompat>& vel_planner,
                                          MultiPointsTrajectory* mpt_ptr);
        ~SASArcTimeNonUniformTraj() = default;

        std::vector<std::vector<double>> getVarySpeedTraj();

       private:
        void initialization();

        void updataTrajectoryTimeSlice(double t);

        void updateSimuTime(bool in_arc);
        void updateTimeRatio(bool in_arc);

        void calTrajPose(size_t id, double ratio);

        std::shared_ptr<SAS::PiecewiseSegments> ps_sptr_;
        std::shared_ptr<VelocityPlannerCompat> vel_planner_;

        MultiPointsTrajectory* mpt_ptr_;

        std::vector<double> dis_vec_;
        std::vector<std::vector<double>> data_;

        // seg info
        std::pair<size_t, SAS::SegType> new_si{0, SAS::SegType::UNKNOWN};
        std::pair<size_t, SAS::SegType> old_si{0, SAS::SegType::UNKNOWN};

        std::vector<std::vector<double>> val_;

        double t_simu_ = 0, t_real_ = 0;

        double end_t_;

        double dt_;

        double traj_total_time = 0.0;
        double time_ratio_     = 1.0;

        int cur_id_  = 0;
        bool in_arc_ = false;

        int tsc_ = 0;

        std::shared_ptr<VariableTimeSlice> vts_;

        size_t num_;

        const double EXP_RATIO   = 0.95;
        const double VAR_RATIO   = -0.01;
        const double DEF_RATIO   = 1.0;
        const double PLANNER_DIM = 6;
    };

}  // namespace vp::tp

#endif