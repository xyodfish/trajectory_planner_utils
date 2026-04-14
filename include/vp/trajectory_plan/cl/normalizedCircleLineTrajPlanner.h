#ifndef __NORMALIZED_CIRCLE_LINE_TRAJ_PLANNER_H__
#define __NORMALIZED_CIRCLE_LINE_TRAJ_PLANNER_H__

#include <memory>
#include "circle_line_traj_planner_interface.h"

namespace vp::tp {
    class NormalizedCircleLineTrajPlanner : public CircleLineTrajPlannerInterface {

       private:
        class NCLTPImpl;
        std::unique_ptr<NCLTPImpl> pImpl_;

       public:
        explicit NormalizedCircleLineTrajPlanner() = default;
        explicit NormalizedCircleLineTrajPlanner(const std::string& _config_file, const std::string& _algo,
                                                 const std::vector<double>& params = {});
        ~NormalizedCircleLineTrajPlanner();

        virtual std::vector<std::vector<double>> getRawTraj() override;
        virtual std::vector<std::vector<double>> getTrajs() override;
        virtual std::vector<std::vector<double>> getPathSegmentsSgPose() override;
    };
}  // namespace vp::tp

#endif