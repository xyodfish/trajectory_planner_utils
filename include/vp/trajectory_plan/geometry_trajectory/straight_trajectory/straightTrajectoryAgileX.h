#ifndef __STRIGHT_TRAJECTORY_AGILE_X_H__
#define __STRIGHT_TRAJECTORY_AGILE_X_H__

#include <memory>
#include <string>
#include <vector>

namespace vp::tp {
    class StraightTrajectoryAgileX {
       private:
        class STAXImpl;
        std::unique_ptr<STAXImpl> pImpl_;

       public:
        StraightTrajectoryAgileX() = default;
        explicit StraightTrajectoryAgileX(const std::vector<std::vector<double>>& sg_pose, std::vector<double>& Bcs);
        virtual ~StraightTrajectoryAgileX();

        std::vector<std::vector<double>> getTrajs();

        double getTrajVel();
        double getTrajAcc();

        double getAverageVel();
    };
}  // namespace vp::tp

#endif