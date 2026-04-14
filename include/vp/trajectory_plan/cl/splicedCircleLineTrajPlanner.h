#ifndef __SPLICING_TRAJECTORY_GENERATION_H__
#define __SPLICING_TRAJECTORY_GENERATION_H__

#include <memory>
#include "VelocityPlannerCompat.h"
#include "circle_line_traj_planner_interface.h"
#include "traj_circle_line.h"

namespace vp::tp {
    class SplicedCircleLineTrajPlanner : public CircleLineTrajPlannerInterface {

       private:
        class SCLTPImpl;
        std::unique_ptr<SCLTPImpl> pImpl_;

       public:
        explicit SplicedCircleLineTrajPlanner() = default;
        explicit SplicedCircleLineTrajPlanner(const std::string& _config_file, const std::string& _algo = "DSVP",
                                              const std::vector<double>& _params = {});
        ~SplicedCircleLineTrajPlanner();

        virtual std::vector<std::vector<double>> getRawTraj() override;
        virtual std::vector<std::vector<double>> getTrajs() override;

        virtual std::vector<std::vector<double>> getPathSegmentsSgPose() override;
    };
}  // namespace vp::tp

#endif