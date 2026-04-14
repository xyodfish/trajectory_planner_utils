#include "multiPointsTrajectory.h"
#include "common_functions.h"
#include "dbg.h"
#include "motionTransform.h"
#include "sasArcTimeNonUniformTraj.h"
#include "sasTimeUniformTraj.h"
#include "spdlog/spdlog.h"

namespace vp::tp {

    MultiPointsTrajectory::~MultiPointsTrajectory() = default;

    MultiPointsTrajectory::MultiPointsTrajectory(const std::vector<std::vector<double>>& points, double cont_dis_coeff)
        : tut_pimpl_(std::make_unique<SASTimeUniformTraj>(points, cont_dis_coeff)) {}

    void MultiPointsTrajectory::initialization() {
        tut_pimpl_->initialization();
    }

    void MultiPointsTrajectory::setWayPoints(const std::vector<std::vector<double>>& points) {
        tut_pimpl_->setWayPoints(points);
    }

    std::vector<BCs<double>>& MultiPointsTrajectory::getBoundaryConditions() {
        return tut_pimpl_->getBoundaryConditions();
    }

    std::string MultiPointsTrajectory::getAlgoName() {
        return tut_pimpl_->getAlgoName();
    }

    void MultiPointsTrajectory::setPlanner(const std::vector<std::vector<double>>& points, double cont_dis_coeff) {

        setWayPoints(points);
        initialization();

        auto BC           = tut_pimpl_->getBoundaryConditions();
        BC[0].g_state.pos = getSegLength();
        tut_pimpl_->setVelPlanner(BC);
    }

    void MultiPointsTrajectory::setPlanner(const std::vector<BCs<double>>& BCs_, const std::string& alg,
                                           const std::shared_ptr<Vpb3d>& vp_params) {
        if (BCs_.size() != 1) {
            spdlog::error("input param size error!  input boundary conditions size {} is not match", BCs_.size());
            return;
        }

        initialization();

        auto BC           = BCs_;
        BC[0].g_state.pos = getSegLength();
        tut_pimpl_->setVelPlanner(BC, alg, vp_params);
    }

    std::vector<double> MultiPointsTrajectory::getBlendRadius() const {
        return tut_pimpl_->getBlendRadius();
    }

    double MultiPointsTrajectory::getSegLength() {
        return tut_pimpl_->getSegLength();
    }

    std::vector<std::vector<double>> MultiPointsTrajectory::getRawTrajs() {
        return tut_pimpl_->getRawTrajs();
    }

    std::vector<std::vector<double>> MultiPointsTrajectory::getTrajs() {
        return tut_pimpl_->getTrajs();
    }

    void MultiPointsTrajectory::appendTrajData(double t, std::vector<std::vector<double>>& val, double time, double dt,
                                               double id) {

        return tut_pimpl_->appendTrajData(t, val, time, dt, id);
    }

    std::vector<std::vector<double>> MultiPointsTrajectory::getVarySpeedTraj() {

        if (!svt_pimpl_.get()) {
            svt_pimpl_ = std::make_unique<SASArcTimeNonUniformTraj>(tut_pimpl_->getPiecewiseSegments(),
                                                                    tut_pimpl_->getVelPlanner(), this);
        }

        return svt_pimpl_->getVarySpeedTraj();
    }
}  // namespace vp::tp