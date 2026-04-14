#ifndef __CIRCLE_LINE_TRAJ_GENERATION_H__
#define __CIRCLE_LINE_TRAJ_GENERATION_H__

#include <memory>
#include <string>
#include <vector>

namespace vp::tp {
    class CircleLineTrajGeneration {

       private:
        class CLTGImpl;
        std::unique_ptr<CLTGImpl> pImpl_;

       public:
        ///
        explicit CircleLineTrajGeneration() = default;
        explicit CircleLineTrajGeneration(const std::string& _path);
        ~CircleLineTrajGeneration();

        /**
         * @brief 
         * 
         * @return std::vector<std::vector<double>> 
         */
        std::vector<std::vector<double>> getCLTrajOnWp();

        /**
         * @brief 
         * 
         * @return std::vector<std::vector<double>> 
         */
        std::vector<std::vector<double>> getCLTrajOnBase();

        /**
         * @brief Get the Teach Points Name object
         * 
         * @return std::vector<std::string> 
         */
        std::string getWpName();

        void setWpPt(const std::vector<double>& _tp);

        void plotTraj(bool on_wp = false);

        void setTrajSpPose(const std::vector<double>& start_pt, bool on_wp = true);  // 获得轨迹起始点在工台下的表达

        std::vector<std::vector<double>> getSegmentsSgPose();

        /**
         * @brief Get the Traj From Data 从文件数模中读到的文件原始点生成的轨迹
         * 
         * @return std::vector<std::vector<double>> 
         */
        std::vector<std::vector<double>> getTrajFromData();
    };
}  // namespace vp::tp

#endif