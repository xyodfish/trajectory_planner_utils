#include "circleLineTrajGeneration.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <filesystem>
#include "VelocityPlannerCompat.h"
#include "common_functions.h"
#include "load_yaml.h"
#include "motionTransform.h"
#include "normalizedCircleLineTrajPlanner.h"
#include "spdlog/spdlog.h"
#include "splicedCircleLineTrajPlanner.h"
#include "traj_circle_line.h"

#ifdef VP_TP_ENABLE_PLOTTING
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

namespace vp::tp {
    using CLTG = CircleLineTrajGeneration;

    // implemention
    class CLTG::CLTGImpl {
       public:
        CLTGImpl() = default;
        explicit CLTGImpl(const std::string& _path);
        ~CLTGImpl() = default;

        /**
         * @brief Get the Teach Pts Name object
         * 
         * @return std::string 
         */
        std::string getWpName();

        /**
         * @brief Set the Wp Pt object
         * 
         * @param _tp 
         */
        void setWpPt(const std::vector<double>& _tp);

        /**
         * @brief 
         * 
         * @return std::vector<std::vector<double>> 
         */
        std::vector<std::vector<double>> getCLTrajOnWp();

        /**
         * @brief 获取基于base下的circle line 轨迹
         * 
         * @return std::vector<std::vector<double>> 
         */
        std::vector<std::vector<double>> getCLTrajOnBase();

        /**
         * @brief Get the Traj From Data object
         * 
         * @return std::vector<std::vector<double>> 
         */
        std::vector<std::vector<double>> getTrajFromData() {
            if (trajFromData_.empty()) {
                trajFromData_ = cltp_sptr_->getTrajs();
            }

            return trajFromData_;
        }

        void plotTraj(bool on_wp);

        void setTrajSpPose(const std::vector<double>& start_pt, bool on_wp);

        std::vector<std::vector<double>> getSegmentsSgPose() { return pathSgPoses_; }

       private:
        std::string resolvePathFromConfig(const std::string& rawPath) const;
        void initConfig();

        void dataClear();

        void updateCLTrajOnWp();

        void updateCLTrajOnBase();

        std::shared_ptr<CircleLineTrajPlannerInterface> createCLTP();

        std::string path_, wp_name_, cltSegFile_, vpAlgo_, splicedCfgFile_;
        std::string cltpType_;
        double max_vel_, max_acc_, max_jerk_, dt_;

        std::vector<double> wpPt_;
        Eigen::Isometry3d T_wp_2_base_, T_sp_to_wp_;

        std::vector<std::vector<double>> trajOnWp_;
        std::vector<std::vector<double>> trajOnBase_;
        std::vector<std::vector<double>> trajFromData_;

        std::vector<double> sp_on_base_, sp_on_wp_;

        std::vector<std::vector<double>> pathSgPoses_;

        std::shared_ptr<CircleLineTrajPlannerInterface> cltp_sptr_;

        YAML::Node yaml_node_;
    };

    CLTG::CLTGImpl::CLTGImpl(const std::string& _path) : path_(_path) {
        initConfig();
        spdlog::info("finish cltg constrution");
    }

    std::string CLTG::CLTGImpl::resolvePathFromConfig(const std::string& rawPath) const {
        std::filesystem::path target(rawPath);
        if (target.is_relative()) {
            target = std::filesystem::path(path_).parent_path() / target;
        }
        return target.lexically_normal().string();
    }

    std::shared_ptr<CircleLineTrajPlannerInterface> CLTG::CLTGImpl::createCLTP() {
        if (cltpType_ == "Normalized") {
            return std::make_shared<NormalizedCircleLineTrajPlanner>(
                cltSegFile_, vpAlgo_, std::vector<double>{max_vel_, max_acc_, max_jerk_, dt_});
        } else if (cltpType_ == "Spliced") {
            return std::make_shared<SplicedCircleLineTrajPlanner>(
                splicedCfgFile_, vpAlgo_, std::vector<double>{max_vel_, max_acc_, max_jerk_, dt_});
        } else {
            spdlog::error("no correct cl traj generation type is defined");
            return nullptr;
        }
    }

    void CLTG::CLTGImpl::initConfig() {

        yaml_node_ = YAML::LoadFile(path_);

        max_vel_  = yaml_node_["max_vel"].as<double>();
        max_acc_  = yaml_node_["max_acc"].as<double>();
        max_jerk_ = yaml_node_["max_jerk"].as<double>();
        dt_       = yaml_node_["dt"].as<double>();

        cltpType_   = yaml_node_["cltp_type"].as<std::string>();
        cltSegFile_ = resolvePathFromConfig(yaml_node_["seg_file_path"].as<std::string>());

        if (cltpType_ == "Spliced") {
            splicedCfgFile_ = resolvePathFromConfig(yaml_node_["spliced_config_file"].as<std::string>());
        }

        vpAlgo_  = yaml_node_["vpAlgo_name"].as<std::string>();
        wp_name_ = yaml_node_["tp_wp_name"].as<std::string>();

        cltp_sptr_ = createCLTP();

        T_wp_2_base_ = Eigen::Isometry3d::Identity();  // 先初始化一个单位矩阵
        T_sp_to_wp_  = Eigen::Isometry3d::Identity();  // 先初始化一个单位矩阵

        sp_on_base_.resize(6);
        sp_on_wp_.resize(6);

        pathSgPoses_ = cltp_sptr_->getPathSegmentsSgPose();

        for (const auto& var : pathSgPoses_) {
            dbg(var);
        }
    }

    void CLTG::CLTGImpl::dataClear() {
        trajOnBase_.clear();
        trajOnWp_.clear();
    }

    std::string CLTG::CLTGImpl::getWpName() {
        return wp_name_;
    }

    void CLTG::CLTGImpl::setWpPt(const std::vector<double>& _wpPt) {
        wpPt_        = _wpPt;
        T_wp_2_base_ = vp::math::pose2Homogeneous(vp::math::vectorToEigen(wpPt_));
    }

    void CLTG::CLTGImpl::updateCLTrajOnWp() {
        dataClear();
        trajOnWp_     = cltp_sptr_->getTrajs();
        trajFromData_ = trajOnWp_;
        std::vector<double> pose(6);

        for (auto i = 0; i < trajOnWp_.size(); ++i) {

            for (auto j = 0; j < 6; ++j) {
                pose[j] = trajOnWp_[i][j + 1];
            }
            // pose = vp::math::homogeneous2STDPose(T_sp_to_wp_ * vp::math::pose2Homogeneous(pose));

            for (auto j = 0; j < 6; ++j) {
                pose[j] += sp_on_wp_[j];
            }

            for (auto j = 0; j < 6; ++j) {
                trajOnWp_[i][j + 1] = pose[j];
            }

            /// @brief calculate vel
            if (i >= 1) {
                for (int j = 0; j < 6; ++j) {
                    trajOnWp_[i][j + 7] = (trajOnWp_[i][j + 1] - trajOnWp_[i - 1][j + 1]) / dt_;
                }

            } else {
                for (int j = 0; j < 6; ++j) {
                    trajOnWp_[i][j + 7] = 0;
                }
            }
        }
    }  // namespace vp::tp

    std::vector<std::vector<double>> CLTG::CLTGImpl::getCLTrajOnWp() {

        updateCLTrajOnWp();
        return trajOnWp_;
    }

    void CLTG::CLTGImpl::updateCLTrajOnBase() {
        std::vector<double> basePose(6);
        std::vector<double> wpPose(6);
        std::vector<double> tp(2 * 6 + 1);

        updateCLTrajOnWp();

        for (auto i = 0; i < trajOnWp_.size(); ++i) {

            // get wp pose
            std::copy(trajOnWp_[i].begin() + 1, trajOnWp_[i].begin() + 7, wpPose.begin());
            basePose = vp::math::homogeneous2STDPose(T_wp_2_base_ * vp::math::pose2Homogeneous(wpPose));

            // copy pose to tp
            tp[0] = trajOnWp_[i][0];
            std::copy(basePose.begin(), basePose.end(), tp.begin() + 1);

            trajOnBase_.push_back(tp);

            /// @brief calculate vel
            if (i >= 1) {
                for (int j = 0; j < 6; ++j) {
                    trajOnBase_[i][j + 7] = (trajOnBase_[i][j + 1] - trajOnBase_[i - 1][j + 1]) / dt_;
                }

            } else {
                for (int j = 0; j < 6; ++j) {
                    trajOnBase_[i][j + 7] = 0;
                }
            }
        }
    }

    std::vector<std::vector<double>> CLTG::CLTGImpl::getCLTrajOnBase() {

        updateCLTrajOnBase();
        return trajOnBase_;
    }

    void CLTG::CLTGImpl::plotTraj(bool on_wp) {
#ifndef VP_TP_ENABLE_PLOTTING
        (void)on_wp;
        spdlog::warn("plotting is disabled, enable VP_TP_ENABLE_PLOTTING to use plotTraj()");
        return;
#else
        auto traj = on_wp ? getCLTrajOnWp() : getCLTrajOnBase();

        std::vector<std::vector<double>> plot_pos(2, std::vector<double>{});
        std::vector<std::vector<double>> plot_vel(4, std::vector<double>{});

        for (auto i = 0; i < traj.size(); ++i) {
            plot_pos[0].push_back(traj[i][1]);
            plot_pos[1].push_back(traj[i][2]);

            for (auto j = 0; j < 3; ++j) {
                plot_vel[j].push_back(traj[i][7 + j]);
            }

            plot_vel[3].push_back(
                computeEuclideanNorm(std::vector<double>({plot_vel[0][i], plot_vel[1][i], plot_vel[2][i]})));
        }

        plt::plot(plot_pos[0], plot_pos[1]);
        plt::axis("equal");
        plt::grid(true);
        plt::show();

        for (auto i = 0; i < 4; ++i) {
            plt ::subplot(2, 2, i + 1);
            plt::plot(plot_vel[i]);
        }

        plt::show();
#endif
    }

    void CLTG::CLTGImpl::setTrajSpPose(const std::vector<double>& start_pt, bool on_wp) {

        if (on_wp) {
            sp_on_wp_   = start_pt;  // on wp
            T_sp_to_wp_ = vp::math::pose2Homogeneous(sp_on_wp_);
            sp_on_base_ = vp::math::homogeneous2STDPose(T_wp_2_base_ * T_sp_to_wp_);

        } else {
            sp_on_base_ = start_pt;  // on wp
            sp_on_wp_   = vp::math::homogeneous2STDPose(T_wp_2_base_.inverse() * vp::math::pose2Homogeneous(sp_on_base_));
            T_sp_to_wp_ = vp::math::pose2Homogeneous(sp_on_wp_);
        }
    }

    CLTG::CircleLineTrajGeneration(const std::string& _path) : pImpl_(std::make_unique<CLTGImpl>(_path)) {}
    CLTG::~CircleLineTrajGeneration(){};

    std::string CLTG::getWpName() {
        return pImpl_->getWpName();
    }

    void CLTG::setWpPt(const std::vector<double>& _tp) {
        pImpl_->setWpPt(_tp);
    }

    std::vector<std::vector<double>> CLTG::getCLTrajOnWp() {
        return pImpl_->getCLTrajOnWp();
    }

    std::vector<std::vector<double>> CLTG::getCLTrajOnBase() {
        return pImpl_->getCLTrajOnBase();
    }

    void CLTG::plotTraj(bool on_wp) {
        return pImpl_->plotTraj(on_wp);
    }

    void CLTG::setTrajSpPose(const std::vector<double>& start_pt, bool on_wp) {
        return pImpl_->setTrajSpPose(start_pt, on_wp);
    }

    std::vector<std::vector<double>> CLTG::getSegmentsSgPose() {
        return pImpl_->getSegmentsSgPose();
    }

    std::vector<std::vector<double>> CLTG::getTrajFromData() {
        return pImpl_->getTrajFromData();
    }

}  // namespace vp::tp
