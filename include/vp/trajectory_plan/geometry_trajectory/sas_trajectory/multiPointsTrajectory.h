#ifndef __MULTI_POINTS_TRAJECTORY_H__
#define __MULTI_POINTS_TRAJECTORY_H__

#include "VelocityPlannerCompat.h"
#include "sas_segments.h"

#include "runtimeCommon.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>

namespace vp::tp {

    class MultiPointsTrajectory {

        using Vector6d = Eigen::Matrix<double, 6, 1>;

       public:
        explicit MultiPointsTrajectory() = delete;
        explicit MultiPointsTrajectory(const std::vector<std::vector<double>>& points, double contact_dis_coeff = 0.15);
        ~MultiPointsTrajectory();

        /**
        * @brief Set the Planner object
        * 
        * @param BCs_ 
        * @param alg 
        * @param vp_params 
        */
        void setPlanner(const std::vector<BCs<double>>& BCs_, const std::string& alg,
                        const std::shared_ptr<Vpb3d>& vp_params = std::shared_ptr<Vpb3d>(nullptr));

        /**
         * @brief Set the Planner object
         * 
         * @param points 
         * @param contact_dis_coeff 
         */
        void setPlanner(const std::vector<std::vector<double>>& points, double contact_dis_coeff = 0.15);

        /**
         * @brief Get the Raw Trajs object
         * 
         * @return std::vector<std::vector<double>> 
         */
        std::vector<std::vector<double>> getRawTrajs();

        /**
         * @brief Get the Trajs object
         * 
         * @return std::vector<std::vector<double>> 
         */
        std::vector<std::vector<double>> getTrajs();

        /**
         * @brief Get the Local Variable Trajectory object
         * 
         * @return std::vector<std::vector<double>> 
         */
        std::vector<std::vector<double>> getVarySpeedTraj();

        /**
         * @brief Get the Blend Radius object
         * 
         * @return std::vector<double> 
         */
        std::vector<double> getBlendRadius() const;

        /**
         * @brief Get the Seg Length object
         * 
         * @return double 
         */
        double getSegLength();

        /**
         * @brief Get the Boundary Conditions object
         * 
         * @return std::vector<BCs<double>>& 
         */
        std::vector<BCs<double>>& getBoundaryConditions();

        std::string getAlgoName();

        void appendTrajData(double t, std::vector<std::vector<double>>& val, double time, double dt, double id);

       private:
        void initialization();

        void setWayPoints(const std::vector<std::vector<double>>& points);

       private:
        class SASArcTimeNonUniformTraj;  // sas 多段变速轨迹
        std::unique_ptr<SASArcTimeNonUniformTraj> svt_pimpl_;

        class SASTimeUniformTraj;
        std::unique_ptr<SASTimeUniformTraj> tut_pimpl_;
    };
}  // namespace vp::tp

#endif