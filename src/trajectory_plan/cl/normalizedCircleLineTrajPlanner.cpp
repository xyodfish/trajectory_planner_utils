#include "normalizedCircleLineTrajPlanner.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include "VelocityPlannerCompat.h"
#include "common_functions.h"
// #include "load_yaml.h"
#include "motionTransform.h"
#include "spdlog/spdlog.h"
#include "traj_circle_line.h"
#include <stdexcept>

namespace vp::tp {
    using NCLTP = NormalizedCircleLineTrajPlanner;

    // implemention
    class NCLTP::NCLTPImpl {
       public:
        NCLTPImpl() = default;
        explicit NCLTPImpl(const std::string& _path, const std::string& _algo, const std::vector<double>& _params);
        ~NCLTPImpl() = default;

        std::vector<std::vector<double>> getRawTraj();
        std::vector<std::vector<double>> getTrajs();
        std::vector<std::vector<double>> getPathSegmentsSgPose();

       private:
        void initConfig();
        tcl::Path clPath_;

        std::string path_;

        std::shared_ptr<VelocityPlannerCompat> vel_planner_;

        std::vector<BCs<double>> BC;

        std::vector<double> params_;

        std::string algo_;
    };

    NCLTP::NCLTPImpl::NCLTPImpl(const std::string& _path, const std::string& _algo, const std::vector<double>& _params)
        : path_(_path), algo_(_algo), params_(_params) {
        initConfig();
        spdlog::info("finish ncltp constrution");
    }

    void NCLTP::NCLTPImpl::initConfig() {

        /// @brief plan traj
        BC.resize(1);
        initDefaultBCs(BC, params_[0], params_[1], params_[2], params_[3]);
        /// @brief get path config from file
        tcl::readPathCicleLine(path_, clPath_);
        if (clPath_.getSegmentsNum() == 0 || clPath_.getLength() <= 0.0) {
            throw std::runtime_error("normalized circle-line planner: empty/invalid path segments");
        }

        /// @brief init the vel planner
        BC[0].g_state.pos = clPath_.getLength();
        vel_planner_      = std::make_shared<VelocityPlannerCompat>(BC, algo_);
    }

    std::vector<std::vector<double>> NCLTP::NCLTPImpl::getRawTraj() {
        return vel_planner_->getTrajs(true);
    }

    std::vector<std::vector<double>> NCLTP::NCLTPImpl::getTrajs() {
        std::vector<std::vector<double>> traj;
        std::vector<double> tp(2 * 6 + 1);
        std::vector<double> pose;

        double path_length = clPath_.getLength();

        auto data = getRawTraj();

        for (auto i = 0; i < data.size(); ++i) {
            tp[0] = data[i][0];
            pose  = clPath_.getConfig(data[i][1] * path_length);

            std::copy(pose.begin(), pose.end(), tp.begin() + 1);

            traj.push_back(tp);

            /// @brief calculate vel
            if (i >= 1) {
                for (int j = 0; j < 6; ++j) {
                    traj[i][j + 7] = (traj[i][j + 1] - traj[i - 1][j + 1]) / params_[3];
                }

            } else {
                for (int j = 0; j < 6; ++j) {
                    traj[i][j + 7] = 0;
                }
            }
        }

        return traj;
    }

    std::vector<std::vector<double>> NCLTP::NCLTPImpl::getPathSegmentsSgPose() {
        return clPath_.getPathSegmentsSgPose();
    }

    NCLTP::NormalizedCircleLineTrajPlanner(const std::string& _path, const std::string& _algo,
                                           const std::vector<double>& _params)
        : pImpl_(std::make_unique<NCLTPImpl>(_path, _algo, _params)) {}

    NCLTP::~NormalizedCircleLineTrajPlanner(){};

    std::vector<std::vector<double>> NCLTP::getRawTraj() {
        return pImpl_->getRawTraj();
    }

    std::vector<std::vector<double>> NCLTP::getTrajs() {
        return pImpl_->getTrajs();
    }

    std::vector<std::vector<double>> NCLTP::getPathSegmentsSgPose() {
        return pImpl_->getPathSegmentsSgPose();
    }

}  // namespace vp::tp
