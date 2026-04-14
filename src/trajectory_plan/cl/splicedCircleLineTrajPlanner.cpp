#include "splicedCircleLineTrajPlanner.h"
#include "load_yaml.h"
#include "spdlog/spdlog.h"
#include <filesystem>

namespace vp::tp {
    using SCLTP = SplicedCircleLineTrajPlanner;

    // implemention
    class SCLTP::SCLTPImpl {
       public:
        SCLTPImpl() = default;
        explicit SCLTPImpl(const std::string& _path, const std::string& _algo, const std::vector<double>& _params);
        ~SCLTPImpl() = default;

        std::vector<std::vector<double>> getRawTraj();
        std::vector<std::vector<double>> getTrajs();
        std::vector<std::vector<double>> getPathSegmentsSgPose();

       private:
        std::string resolvePathFromConfig(const std::string& rawPath) const;
        void initConfig();
        void generateVelocityPlanner();

        tcl::Path clPath_;

        std::string path_;
        std::string cltSegFile_;
        std::vector<BCs<double>> BCs_;
        std::vector<std::shared_ptr<VelocityPlannerCompat>> vps_;
        std::vector<double> parmas_;
        std::string algo_;

        class TargetBCSInfo {
           public:
            std::vector<double> max_vel_;
            std::vector<double> max_acc_;
            std::vector<double> max_jerk_;
            std::vector<double> end_vel_;
            std::vector<double> end_acc_;
            std::vector<double> end_jerk_;
            std::vector<double> dt_;
        };

        TargetBCSInfo bcs_;

        std::vector<double> pathLengthVec_;
    };

    SCLTP::SCLTPImpl::SCLTPImpl(const std::string& _path, const std::string& _algo, const std::vector<double>& _params)
        : path_(_path), algo_(_algo), parmas_(_params) {
        initConfig();
        spdlog::info("finish ncltp constrution");
    }

    std::string SCLTP::SCLTPImpl::resolvePathFromConfig(const std::string& rawPath) const {
        std::filesystem::path target(rawPath);
        if (target.is_relative()) {
            target = std::filesystem::path(path_).parent_path() / target;
        }
        return target.lexically_normal().string();
    }

    void SCLTP::SCLTPImpl::initConfig() {
        auto node   = YAML::LoadFile(path_);
        cltSegFile_ = resolvePathFromConfig(node["seg_file_path"].as<std::string>());

        bcs_.max_vel_  = node["BC_Info"]["max_v"].as<std::vector<double>>();
        bcs_.max_acc_  = node["BC_Info"]["max_a"].as<std::vector<double>>();
        bcs_.max_jerk_ = node["BC_Info"]["max_j"].as<std::vector<double>>();
        bcs_.end_vel_  = node["BC_Info"]["end_v"].as<std::vector<double>>();
        bcs_.end_acc_  = node["BC_Info"]["end_a"].as<std::vector<double>>();
        bcs_.end_jerk_ = node["BC_Info"]["end_j"].as<std::vector<double>>();
        bcs_.dt_       = node["BC_Info"]["dt"].as<std::vector<double>>();

        generateVelocityPlanner();
    }

    void SCLTP::SCLTPImpl::generateVelocityPlanner() {
        tcl::readPathCicleLine(cltSegFile_, clPath_);

        BCs_.resize(clPath_.getSegmentsNum());
        vps_.resize(clPath_.getSegmentsNum());

        pathLengthVec_.clear();

        for (auto i = 0; i < clPath_.getSegmentsNum(); ++i) {
            pathLengthVec_.push_back(clPath_.getLength(i));
        }

        BCs_[0].s_state.pos  = 0;
        BCs_[0].s_state.vel  = 0;
        BCs_[0].s_state.acc  = 0;
        BCs_[0].s_state.jerk = 0;

        BCs_[0].g_state.pos = pathLengthVec_[0];
        BCs_[0].g_state.vel = bcs_.end_vel_[0];

        BCs_[0].max_vel  = bcs_.max_vel_[0];
        BCs_[0].max_acc  = bcs_.max_acc_[0];
        BCs_[0].max_jerk = bcs_.max_jerk_[0];
        BCs_[0].delta_t  = bcs_.dt_[0];

        auto cur_bc = std::vector<BCs<double>>{BCs_[0]};
        vps_[0]     = std::make_shared<VelocityPlannerCompat>(cur_bc, algo_);

        for (auto i = 1; i < BCs_.size(); ++i) {

            ///
            auto last_end_traj = vps_[i - 1]->getEndTraj();

            BCs_[i].s_state.pos  = last_end_traj[0];
            BCs_[i].s_state.vel  = last_end_traj[1];
            BCs_[i].s_state.acc  = last_end_traj[2];
            BCs_[i].s_state.jerk = last_end_traj[3];

            BCs_[i].g_state.pos  = BCs_[i].s_state.pos + pathLengthVec_[i];
            BCs_[i].g_state.vel  = bcs_.end_vel_[i];
            BCs_[i].g_state.acc  = bcs_.end_acc_[i];
            BCs_[i].g_state.jerk = bcs_.end_jerk_[i];

            BCs_[i].max_vel  = bcs_.max_vel_[i];
            BCs_[i].max_acc  = bcs_.max_acc_[i];
            BCs_[i].max_jerk = bcs_.max_jerk_[i];
            BCs_[i].delta_t  = bcs_.dt_[i];

            auto cur_bc = std::vector<BCs<double>>{BCs_[i]};
            vps_[i]     = std::make_shared<VelocityPlannerCompat>(cur_bc, "DSVP");
        }
    }

    std::vector<std::vector<double>> SCLTP::SCLTPImpl::getRawTraj() {
        std::vector<std::vector<double>> rawTraj;

        for (const auto& vp : vps_) {

            auto traj = vp->getTrajs(true);

            for (const auto& var : traj) {
                rawTraj.push_back(var);
            }
        }

        return rawTraj;
    }

    std::vector<std::vector<double>> SCLTP::SCLTPImpl::getTrajs() {
        std::vector<std::vector<double>> traj;
        std::vector<double> tp(2 * 6 + 1);
        std::vector<double> pose;

        double path_length = clPath_.getLength();

        for (auto i = 0; i < vps_.size(); ++i) {
            auto data = vps_[i]->getTrajs(true);

            for (auto j = 0; j < data.size(); ++j) {
                tp[0] = data[j][0];
                pose  = clPath_.getConfig(i, data[j][1]);

                if (pose.empty()) {
                    spdlog::warn("get traj pose is empty");
                    continue;
                }

                std::copy(pose.begin(), pose.end(), tp.begin() + 1);

                traj.push_back(tp);
            }
        }

        double dt = traj[1][0] - traj[0][0];
        for (auto i = 0; i < traj.size(); ++i) {
            if (i >= 1) {
                for (int j = 0; j < 6; ++j) {
                    traj[i][j + 7] = (traj[i][j + 1] - traj[i - 1][j + 1]) / dt;
                }

            } else {
                for (int j = 0; j < 6; ++j) {
                    traj[i][j + 7] = 0;
                }
            }
        }

        return traj;
    }

    std::vector<std::vector<double>> SCLTP::SCLTPImpl::getPathSegmentsSgPose() {
        return clPath_.getPathSegmentsSgPose();
    }

    SCLTP::SplicedCircleLineTrajPlanner(const std::string& _path, const std::string& _algo,
                                        const std::vector<double>& _params)
        : pImpl_(std::make_unique<SCLTPImpl>(_path, _algo, _params)) {}

    SCLTP::~SplicedCircleLineTrajPlanner() = default;

    std::vector<std::vector<double>> SCLTP::getRawTraj() {
        return pImpl_->getRawTraj();
    }

    std::vector<std::vector<double>> SCLTP::getTrajs() {
        return pImpl_->getTrajs();
    }

    std::vector<std::vector<double>> SCLTP::getPathSegmentsSgPose() {
        return pImpl_->getPathSegmentsSgPose();
    }
}  // namespace vp::tp
