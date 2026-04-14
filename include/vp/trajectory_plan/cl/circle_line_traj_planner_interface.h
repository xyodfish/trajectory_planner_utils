#ifndef __CIRCLE_LINE_TRAJ_PLANNER_H__
#define __CIRCLE_LINE_TRAJ_PLANNER_H__

#include <string>
#include <vector>

namespace vp::tp {
    class CircleLineTrajPlannerInterface {
       public:
        explicit CircleLineTrajPlannerInterface() = default;
        explicit CircleLineTrajPlannerInterface(const std::string& _path, const std::string& _algo,
                                                const std::vector<double>& _params) {}
        virtual ~CircleLineTrajPlannerInterface() = default;

        virtual std::vector<std::vector<double>> getRawTraj() = 0;
        virtual std::vector<std::vector<double>> getTrajs()   = 0;

        virtual std::vector<std::vector<double>> getPathSegmentsSgPose() = 0;
    };
}  // namespace vp::tp

#endif