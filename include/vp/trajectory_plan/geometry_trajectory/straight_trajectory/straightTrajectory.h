#ifndef _STRAIGHT_TRAJECTORY_H
#define _STRAIGHT_TRAJECTORY_H
#include "VelocityPlannerCompat.h"

#include <Eigen/Core>
#include <Eigen/Dense>
namespace vp::tp {
    class StraightTrajectory {
       public:
        /// @brief 规划类型
        enum PlanType { GIVEN_MULTI_BCS = 0, GIVEN_SG_POINTS, NO_PARAMS_CONS };

        explicit StraightTrajectory() { m_plan_type = NO_PARAMS_CONS; };
        explicit StraightTrajectory(const std::vector<BCs<double>>& BCs_, const std::string& alg);
        StraightTrajectory(std::vector<double> s_pose, std::vector<double> g_pose, std::vector<BCs<double>> BCs_,
                           const std::string& alg);
        ~StraightTrajectory() = default;

        /**
         * @brief Get the Trajs object
         *
         * @return std::vector< std::vector< KState< double > > >
         */
        std::vector<std::vector<KState<double>>> getKStates() const;

        std::vector<std::vector<double>> getTrajs() const;

        /**
         * @brief Set the Planner object
         *
         * @param BCs_
         * @param alg
         */
        void setPlanner(const std::vector<BCs<double>>& BCs_, const std::string& alg);

        void setPlanner(const std::vector<double>& s_pose, const std::vector<double>& g_pose,
                        std::vector<BCs<double>> BCs_, const std::string& alg);

        std::vector<std::vector<double>> getRawTrajs();

       private:
        PlanType m_plan_type;
        std::vector<BCs<double>> m_BCs_;
        std::vector<double> s_pose_, g_pose_;
        std::shared_ptr<VelocityPlannerCompat> vel_planner;
        const unsigned int PLANNER_DIM = 6;
    };

}  // namespace vp::tp

#endif