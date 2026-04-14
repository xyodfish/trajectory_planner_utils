#include "straightTrajectory.h"
#include "dbg.h"
#include "motionTransform.h"
#include "spdlog/spdlog.h"
#include "traj.h"

using Vector6d = Eigen::Matrix<double, 6, 1>;

namespace vp::tp {
    StraightTrajectory::StraightTrajectory(const std::vector<BCs<double>>& BCs_, const std::string& alg)
        : m_BCs_(BCs_) {
        m_plan_type = GIVEN_MULTI_BCS;
        setPlanner(BCs_, alg);
    }

    StraightTrajectory::StraightTrajectory(std::vector<double> s_pose, std::vector<double> g_pose,
                                           std::vector<BCs<double>> BCs_, const std::string& alg)
        : s_pose_(s_pose), g_pose_(g_pose), m_BCs_(BCs_) {
        m_plan_type = GIVEN_SG_POINTS;
        setPlanner(BCs_, alg);
    }

    /**
     * @brief 针对有参构造后 或者 m_plan_type被赋值后 可以调用该接口
     * 
     * @param BCs_ 
     * @param alg 
     */
    void StraightTrajectory::setPlanner(const std::vector<BCs<double>>& BCs_, const std::string& alg) {

        if (((m_plan_type == GIVEN_MULTI_BCS) && (BCs_.size() != PLANNER_DIM)) ||
            ((m_plan_type == GIVEN_SG_POINTS) && (BCs_.size() != 1))) {
            spdlog::error("input param size error!  input boundary conditions size {} is not match the plan type {}",
                          BCs_.size(), (int)m_plan_type);
            return;
        }

        vel_planner = std::make_shared<VelocityPlannerCompat>(BCs_, alg);
    }

    void StraightTrajectory::setPlanner(const std::vector<double>& s_pose, const std::vector<double>& g_pose,
                                        std::vector<BCs<double>> BCs_, const std::string& alg) {
        s_pose_ = s_pose;
        g_pose_ = g_pose;

        vel_planner = std::make_shared<VelocityPlannerCompat>(BCs_, alg);
        m_plan_type = GIVEN_SG_POINTS;
    }

    std::vector<std::vector<KState<double>>> StraightTrajectory ::getKStates() const {
        return vel_planner->getKStates();
    }

    std::vector<std::vector<double>> StraightTrajectory::getTrajs() const {
        ///
        if (m_plan_type == GIVEN_MULTI_BCS) {
            return vel_planner->getTrajs();
        } else {

            double cur_tar_linear_vel_, cur_tar_rotate_vel_;
            double max_tar_linear_vel_ = std::numeric_limits<double>::min();
            double max_tar_rotate_vel_ = std::numeric_limits<double>::min();

            double cur_tar_linear_acc_, cur_tar_rotate_acc_;
            double max_tar_linear_acc_ = std::numeric_limits<double>::min();
            double max_tar_rotate_acc_ = std::numeric_limits<double>::min();

            // 规划一处理
            auto trajs    = vel_planner->getTrajs(true);
            auto traj_num = trajs.size();

            std::vector<std::vector<double>> rTrajs(traj_num, std::vector<double>(PLANNER_DIM * 3 + 1));
            std::vector<double> tmp_pose;

            dbg(trajs[traj_num - 1][1]);

            for (int i = 0; i < traj_num; ++i) {
                tmp_pose     = vp::math::tcpPoseInterpolation(s_pose_, g_pose_, trajs[i][1]);
                rTrajs[i][0] = trajs[i][0];
                std::copy(tmp_pose.begin(), tmp_pose.end(), rTrajs[i].begin() + 1);
            }

            // cal vel
            for (int i = 1; i < traj_num; ++i) {
                for (int j = 0; j < PLANNER_DIM; ++j) {
                    rTrajs[i][j + 1 + PLANNER_DIM] =
                        (rTrajs[i][j + 1] - rTrajs[i - 1][j + 1]) / (rTrajs[i][0] - rTrajs[i - 1][0]);
                }

                cur_tar_linear_vel_ =
                    std::sqrt(rTrajs[i][7] * rTrajs[i][7] + rTrajs[i][8] * rTrajs[i][8] + rTrajs[i][9] * rTrajs[i][9]);

                cur_tar_rotate_vel_ = std::sqrt(rTrajs[i][10] * rTrajs[i][10] + rTrajs[i][11] * rTrajs[i][11] +
                                                rTrajs[i][12] * rTrajs[i][12]);

                if (max_tar_linear_vel_ < cur_tar_linear_vel_) {
                    max_tar_linear_vel_ = cur_tar_linear_vel_;
                }

                if (max_tar_rotate_vel_ < cur_tar_rotate_vel_) {
                    max_tar_rotate_vel_ = cur_tar_rotate_vel_;
                }
            }

            for (int i = 1; i < traj_num; ++i) {
                for (int j = 0; j < PLANNER_DIM; ++j) {
                    rTrajs[i][j + 1 + 2 * PLANNER_DIM] =
                        (rTrajs[i][j + 1 + PLANNER_DIM] - rTrajs[i - 1][j + 1 + PLANNER_DIM]) /
                        (rTrajs[i][0] - rTrajs[i - 1][0]);
                }

                cur_tar_linear_acc_ = std::sqrt(rTrajs[i][13] * rTrajs[i][13] + rTrajs[i][14] * rTrajs[i][14] +
                                                rTrajs[i][15] * rTrajs[i][15]);

                cur_tar_rotate_acc_ = std::sqrt(rTrajs[i][16] * rTrajs[i][16] + rTrajs[i][17] * rTrajs[i][17] +
                                                rTrajs[i][18] * rTrajs[i][18]);

                if (max_tar_linear_acc_ < cur_tar_linear_acc_) {
                    max_tar_linear_acc_ = cur_tar_linear_acc_;
                }

                if (max_tar_rotate_acc_ < cur_tar_rotate_acc_) {
                    max_tar_rotate_acc_ = cur_tar_rotate_acc_;
                }
            }

            spdlog::info("the vel planner max linear vel is {}, max rotate vel is {}", max_tar_linear_vel_,
                         max_tar_rotate_vel_);
            spdlog::info("the vel planner max linear acc is {}, max rotate acc is {}", max_tar_linear_acc_,
                         max_tar_rotate_acc_);

            return rTrajs;
        }
    }

    std::vector<std::vector<double>> StraightTrajectory::getRawTrajs() {
        return vel_planner->getTrajs(true);
    }
}  // namespace vp::tp
