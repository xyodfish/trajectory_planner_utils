#include "VelocityPlannerCompat.h"

#include <memory>
#include <tuple>

#include "vp/multi_velocity_planner.h"

namespace vp::tp {
namespace compat_detail {

vp::BCs<double> toVpBC(const BCs<double>& bc) {
    vp::BCs<double> out;
    out.start_state.time = bc.s_state.time;
    out.start_state.pos = bc.s_state.pos;
    out.start_state.vel = bc.s_state.vel;
    out.start_state.acc = bc.s_state.acc;
    out.start_state.jerk = bc.s_state.jerk;
    out.start_state.theta = bc.s_state.theta;

    out.goal_state.time = bc.g_state.time;
    out.goal_state.pos = bc.g_state.pos;
    out.goal_state.vel = bc.g_state.vel;
    out.goal_state.acc = bc.g_state.acc;
    out.goal_state.jerk = bc.g_state.jerk;
    out.goal_state.theta = bc.g_state.theta;

    out.max_acc = bc.max_acc;
    out.max_vel = bc.max_vel;
    out.max_jerk = bc.max_jerk;
    out.delta_t = bc.delta_t;
    return out;
}

std::vector<vp::BCs<double>> toVpBCs(const std::vector<BCs<double>>& bcs) {
    std::vector<vp::BCs<double>> out;
    out.reserve(bcs.size());
    for (const auto& bc : bcs) {
        out.push_back(toVpBC(bc));
    }
    return out;
}

std::shared_ptr<vp::VpParams3D> toVpParams(const std::shared_ptr<Vpb3d>& params) {
    if (!params) {
        return nullptr;
    }
    auto out = std::make_shared<vp::VpParams3D>();
    out->params = std::make_tuple(std::get<0>(params->params),
                                  std::get<1>(params->params),
                                  std::get<2>(params->params));
    return out;
}

KState<double> fromVpKState(const vp::KinematicState<double>& s) {
    KState<double> out;
    out.time = s.time;
    out.pos = s.pos;
    out.vel = s.vel;
    out.acc = s.acc;
    out.jerk = s.jerk;
    out.theta = s.theta;
    return out;
}

}  // namespace compat_detail

class VelocityPlannerCompat::Impl {
   public:
    explicit Impl(const std::vector<BCs<double>>& BCs, const std::string& algs)
        : planner_(compat_detail::toVpBCs(BCs), algs) {}

    explicit Impl(const std::vector<BCs<double>>& BCs,
                  const std::string& algs,
                  const std::shared_ptr<Vpb3d>& params)
        : planner_(compat_detail::toVpBCs(BCs), algs, compat_detail::toVpParams(params)) {}

    vp::MultiVelocityPlanner planner_;
};

VelocityPlannerCompat::VelocityPlannerCompat(const std::vector<BCs<double>>& BCs,
                                     const std::string& algs)
    : impl_(std::make_unique<Impl>(BCs, algs)) {}

VelocityPlannerCompat::VelocityPlannerCompat(const std::vector<BCs<double>>& BCs,
                                     const std::string& algs,
                                     const std::shared_ptr<Vpb3d>& params)
    : impl_(std::make_unique<Impl>(BCs, algs, params)) {}

VelocityPlannerCompat::~VelocityPlannerCompat() = default;

std::vector<std::vector<KState<double>>> VelocityPlannerCompat::getKStates(bool isNormalized) {
    auto src = impl_->planner_.getKStates(isNormalized);
    std::vector<std::vector<KState<double>>> out;
    out.reserve(src.size());
    for (const auto& dof : src) {
        std::vector<KState<double>> row;
        row.reserve(dof.size());
        for (const auto& s : dof) {
            row.push_back(compat_detail::fromVpKState(s));
        }
        out.push_back(std::move(row));
    }
    return out;
}

std::vector<std::vector<double>> VelocityPlannerCompat::getTrajs(bool isNormalized) {
    return impl_->planner_.getTrajs(isNormalized);
}

std::vector<std::vector<double>> VelocityPlannerCompat::getPosVec() {
    return impl_->planner_.getPosVec();
}

std::vector<double> VelocityPlannerCompat::getEndTraj(bool isNormalized) {
    return impl_->planner_.getEndTraj(isNormalized);
}

void initDefaultBCs(std::vector<BCs<double>>& BCs, double vel, double acc, double jerk, double dt) {
    for (auto& bc : BCs) {
        bc.s_state.pos = 0.0;
        bc.g_state.pos = 0.0;

        bc.s_state.vel = 0.0;
        bc.s_state.acc = 0.0;
        bc.s_state.jerk = 0.0;

        bc.g_state.vel = 0.0;
        bc.g_state.acc = 0.0;
        bc.g_state.jerk = 0.0;

        bc.max_vel = vel;
        bc.max_acc = acc;
        bc.max_jerk = jerk;
        bc.delta_t = dt;
    }
}

}  // namespace vp::tp
