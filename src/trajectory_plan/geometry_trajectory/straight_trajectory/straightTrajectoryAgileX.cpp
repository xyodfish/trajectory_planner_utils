#include "straightTrajectoryAgileX.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include "VelocityPlannerCompat.h"
#include "common_functions.h"
#include "motionTransform.h"
#include "spdlog/spdlog.h"

namespace vp::tp {

    using STAX = StraightTrajectoryAgileX;

    // implemention
    class STAX::STAXImpl {
       public:
        STAXImpl() = default;
        explicit STAXImpl(const std::vector<std::vector<double>> sg_pose, std::vector<double>& Bcs);
        ~STAXImpl() = default;

        std::vector<std::vector<double>> getTrajs();

        double getTrajVel();
        double getTrajAcc();
        double getAverageVel();

       private:
        std::vector<double> s_pose_, g_pose_;
        std::shared_ptr<VelocityPlannerCompat> vel_planner;

        const unsigned int PLANNER_DIM = 6;

        double real_max_vel_ = 0;
        double real_max_acc_ = 0;
        double real_avg_vel_ = 0;

        double dt_ = 0;
    };

    double STAX::STAXImpl::getTrajVel() {
        return real_max_vel_;
    }

    double STAX::STAXImpl::getTrajAcc() {
        return real_max_acc_;
    }

    double STAX::STAXImpl::getAverageVel() {
        return real_avg_vel_;
    }

    STAX::STAXImpl::STAXImpl(const std::vector<std::vector<double>> sg_pose, std::vector<double>& Bcs) {
        s_pose_ = sg_pose[0];
        g_pose_ = sg_pose[1];

        std::vector<BCs<double>> BC;
        BC.resize(1);
        initDefaultBCs(BC, Bcs[0], Bcs[1], Bcs[2], Bcs[3]);
        dt_ = Bcs[3];

        BC[0].g_state.pos = vp::math::twoVectorDistance(s_pose_, g_pose_);
        vel_planner       = std::make_shared<VelocityPlannerCompat>(BC, "TVP");
    }

    std::vector<std::vector<double>> STAX::STAXImpl::getTrajs() {

        double cur_tar_linear_vel_, cur_tar_rotate_vel_;
        double max_tar_linear_vel_ = std::numeric_limits<double>::min();
        double max_tar_rotate_vel_ = std::numeric_limits<double>::min();

        double cur_tar_linear_acc_, cur_tar_rotate_acc_;
        double max_tar_linear_acc_ = std::numeric_limits<double>::min();
        double max_tar_rotate_acc_ = std::numeric_limits<double>::min();

        // 规划一处理
        auto trajs    = vel_planner->getTrajs(true);
        auto traj_num = trajs.size();

        double duration = (trajs[1][0] - trajs[0][0]) * traj_num;
        double distance = vp::math::twoVectorDistance(s_pose_, g_pose_);

        real_avg_vel_ = distance / duration;

        std::vector<std::vector<double>> rTrajs(traj_num, std::vector<double>(PLANNER_DIM * 3 + 1));
        std::vector<double> tmp_pose;

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

        real_max_vel_ = max_tar_linear_vel_;

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

        real_max_acc_ = max_tar_linear_acc_;

        spdlog::info("the vel planner max linear vel is {}, max rotate vel is {}", max_tar_linear_vel_,
                     max_tar_rotate_vel_);
        spdlog::info("the vel planner max linear acc is {}, max rotate acc is {}", max_tar_linear_acc_,
                     max_tar_rotate_acc_);

        return rTrajs;
    }

    STAX::StraightTrajectoryAgileX(const std::vector<std::vector<double>>& sg_pose, std::vector<double>& Bcs)
        : pImpl_(std::make_unique<STAXImpl>(sg_pose, Bcs)) {}
    STAX::~StraightTrajectoryAgileX(){};

    std::vector<std::vector<double>> STAX::getTrajs() {
        return pImpl_->getTrajs();
    }

    double STAX::getTrajAcc() {
        return pImpl_->getTrajAcc();
    }

    double STAX::getTrajVel() {
        return pImpl_->getTrajVel();
    }

    double STAX::getAverageVel() {
        return pImpl_->getAverageVel();
    }

};  // namespace vp::tp
