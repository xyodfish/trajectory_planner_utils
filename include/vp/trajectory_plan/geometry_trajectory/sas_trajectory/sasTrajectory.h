#ifndef __STRAIGHT_ARC_STRAIGHT__NEW_H__
#define __STRAIGHT_ARC_STRAIGHT__NEW_H__

#include "VelocityPlannerCompat.h"
#include "sas_segments.h"

#include "runtimeCommon.h"

#include <Eigen/Core>
#include <Eigen/Dense>

namespace vp::tp {

    class SASTrajectory {
        using Vector6d = Eigen::Matrix<double, 6, 1>;

       public:
        explicit SASTrajectory(const std::vector<double>& s_pose, const std::vector<double>& m_pose,
                               const std::vector<double>& g_pose, double contact_dis_coeff = 0.15);
        explicit SASTrajectory(const std::vector<double>& s_pose, const std::vector<double>& m_pose,
                               const std::vector<double>& g_pose, std::vector<BCs<double>> BCs_, const std::string& alg,
                               double contact_dis_coeff = 0.15);
        ~SASTrajectory() = default;
        /**
         * @brief Get the Trajs object
         *
         * @return std::vector< std::vector< KState< double > > >
         */
        std::vector<std::vector<KState<double>>> getKStates() const;

        std::vector<std::vector<double>> getTrajs();

        std::vector<std::vector<double>> getRawTrajs();

        Vector6d caculatePoseOnArc(double t);

        /**
         * @brief Set the Planner object
         *
         * @param BCs_
         * @param alg
         */
        void setPlanner(const std::vector<BCs<double>>& BCs_, const std::string& alg);

        void setPlanner(const std::vector<double>& s_pose, const std::vector<double>& m_pose,
                        const std::vector<double>& g_pose, double contact_dis_coeff = 0.15);

        void setPlanner(const std::vector<double>& s_pose, const std::vector<double>& m_pose,
                        const std::vector<double>& g_pose, std::vector<BCs<double>> BCs_, const std::string& alg,
                        double contact_dis_coeff = 0.15);

       private:
        void initialization();

        std::shared_ptr<SAS::PiecewiseSegments> ps_sptr;
        std::vector<std::shared_ptr<SAS::SASegment>> seg_sptr;

        std::vector<BCs<double>> m_BCs_;
        std::shared_ptr<VelocityPlannerCompat> vel_planner;
        const unsigned int PLANNER_DIM = 6;

        std::string m_alg;

        Vector6d p1_, p2_, p3_;
        Vector6d p_con1_, p_con2_;

        double contact_dis_coeff_;

        enum PlanType { GIVEN_MULTI_BCS = 0, GIVEN_POINTS };

        PlanType m_plan_type;

        Eigen::Vector3d v1_, v2_, v3_;
        Eigen::Vector3d m_center_, m_con_p1_, m_con_p2_;
        Eigen::Vector3d m_planeNormal_;

        Eigen::Vector3d m_dis_coeff_;

        Eigen::Matrix3d R_;

        Eigen::Quaterniond q1_, q2_, q3_;

        double m_con_dis_;  //  切点到中间点的距离
        double m_angle_;    // 三点夹角
        double m_angle_2_;
        double m_radius_;
        double m_rad_, m_arc_dis_;
        double m_traj_dis_;

        double m_con1_coeff_, m_con2_coeff_;
    };
}  // namespace vp::tp

#endif