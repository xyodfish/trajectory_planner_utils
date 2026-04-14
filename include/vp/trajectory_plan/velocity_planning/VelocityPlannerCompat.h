#ifndef VP_TP_VELOCITY_PLANNER_COMPAT_H
#define VP_TP_VELOCITY_PLANNER_COMPAT_H

#include <memory>
#include <string>
#include <vector>

#include "velocity_planner_interface.h"

namespace vp::tp {

class VelocityPlannerCompat : public VelocityPlannerCompatInterface<double> {
   public:
    explicit VelocityPlannerCompat(const std::vector<BCs<double>>& BCs, const std::string& algs);
    explicit VelocityPlannerCompat(const std::vector<BCs<double>>& BCs,
                                   const std::string& algs,
                                   const std::shared_ptr<Vpb3d>& params);

    ~VelocityPlannerCompat() override;

    std::vector<std::vector<KState<double>>> getKStates(bool isNormalized = false) override;

    std::vector<std::vector<double>> getTrajs(bool isNormalized = false) override;

    std::vector<std::vector<double>> getPosVec() override;

    std::vector<double> getEndTraj(bool isNormalized = false);

   private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

void initDefaultBCs(std::vector<BCs<double>>& BCs, double vel, double acc, double jerk, double dt);

}  // namespace vp::tp

#endif  // VP_TP_VELOCITY_PLANNER_COMPAT_H
