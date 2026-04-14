#ifndef __SAS_GENERATION_IMPL_H__
#define __SAS_GENERATION_IMPL_H__

#include "VelocityPlannerCompat.h"
#include "multiPointsTrajectory.h"
#include "sas_segments.h"

namespace vp::tp {

    using Vector6d = Eigen::Matrix<double, 6, 1>;

    class MultiPointsTrajectory::SASTimeUniformTraj {

       public:
        explicit SASTimeUniformTraj() = delete;
        explicit SASTimeUniformTraj(const std::vector<std::vector<double>>& m_points, const double& cont_dis_coeff);
        ~SASTimeUniformTraj() = default;

        void initialization();

        std::shared_ptr<SAS::PiecewiseSegments>& getPiecewiseSegments();

        std::shared_ptr<VelocityPlannerCompat>& getVelPlanner();

        std::vector<double> getBlendRadius() const;

        double getSegLength() const;

        std::vector<BCs<double>>& getBoundaryConditions() { return m_BCs; }

        std::string getAlgoName() { return m_alg; }

        std::vector<std::vector<double>> getRawTrajs();

        std::vector<std::vector<double>> getTrajs();

        std::vector<double> getInterpolationPose(double t);

        void setWayPoints(const std::vector<std::vector<double>>& points);

        void setVelPlanner(const std::vector<BCs<double>>& BCs);

        void setVelPlanner(const std::vector<BCs<double>>& BCs, const std::string& algo);

        void setVelPlanner(const std::vector<BCs<double>>& BCs, const std::string& algo,
                           const std::shared_ptr<Vpb3d>& params);

        void appendTrajData(double t, std::vector<std::vector<double>>& val, double time, double dt, double id);

       private:
        void init_params();

        double calculateCutInDistance(double cut_in_coeff);

        void solveSASValue(double cut_in_dis);

        void addPiecewiseSegments(const std::vector<std::pair<Vector6d, Vector6d>>& sl_pts,
                                  const std::vector<SAS::ArcValClass>& arc_pts);

        void calculateVelAndAcc(std::vector<std::vector<double>>& val, size_t id, double dt);

        std::shared_ptr<VelocityPlannerCompat> vel_planner;

        std::shared_ptr<SAS::PiecewiseSegments> ps_sptr;

        std::vector<std::vector<double>> m_points_;
        std::vector<Vector6d> m_pts_;
        double cont_dis_coeff_;

        size_t segNum_, piecewiseNum_;

        std::vector<std::shared_ptr<SAS::SASegment>> seg_sptr;

        std::vector<double> blendRadius_;

        std::vector<BCs<double>> m_BCs;

        std::string m_alg;
    };
}  // namespace vp::tp
#endif