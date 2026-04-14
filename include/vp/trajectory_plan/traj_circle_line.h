/**
 * @file traj_circle_line.h
 * @author original.author@example.com
 * @brief  直线＋圆弧 轨迹规划算法，此文件涉及算法由TOTG算法path部分裁剪、修改后得到，
 * 将6D路径（直线、圆弧）按弧长平展至1维，随后使用梯形速度轨迹规划，路径中各点姿态由其
 * 对应路径段起终点姿态插值得到
 * @version 0.1
 * @date 2023-11-01
 *
 * @copyright Copyright (c) 2023 Original Source
 */

#ifndef TRAJ_CIRCLE_LINE_H_
#define TRAJ_CIRCLE_LINE_H_

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include "motionTransform.h"

namespace vp::tp {
    /// traj circle line
    namespace tcl {
        /// @brief  6D pose
        using Vector6d = Eigen::Matrix<double, 6, 1>;

        /**
         * @brief Eigen::Matrix<double, 6, 1> to std::vector<double>(6)
         * @param vec 6D pose
         * @return std::vector<double>
         */
        std::vector<double> Vector6dToSTD(const Vector6d& vec);

        /// @brief 轨迹片段
        class PathSegment {
           public:
            /// @brief 路径片段类型
            enum SegmentType { UNKNOWN = 0, LINEAR, CIRCULAR };

            /**
             * @brief 路径片段类构造函数
             * @param start 路径起点位姿
             * @param end 路径终点位姿
             * @param length 路径片段长度（弧长）
             */
            explicit PathSegment(Vector6d start, Vector6d end, double length = 0.0)
                : start_(start),
                  end_(end),
                  start_q_(vp::math::getQuaternionFromTcpPose(start)),
                  end_q_(vp::math::getQuaternionFromTcpPose(end)),
                  length_(length) {}
            virtual ~PathSegment() = default;

            /**
             * @brief 获取路径片段长度
             * @return double 路径片段长度
             */
            double getLength() const { return length_; }

            /**
             * @brief 获取起点位姿
             * @return 起点位姿
             */
            const Vector6d& getStart() const { return start_; }

            /**
             * @brief 获取终点位姿
             * @return 终点位姿
             */
            const Vector6d& getEnd() const { return end_; }

            /**
             * @brief 获取路径片段距起点指定弧长处位姿
             * @param s 期望处位姿距起点弧长
             * @return std::vector<double> 路径片段距起点指定弧长处位姿
             */
            virtual std::vector<double> getConfig(double s) const = 0;

           protected:
            /**
             * @brief 获取路径片段指定长度比例处姿态（轴×角形式）。本算法将圆弧、直线路径平展为一条直线，各路径段上各轨迹点姿态由路径段起点、终点插值得到。
             * @param t 期望处位姿距起点弧长比例 t = s / length
             * @return Eigen::Vector3d 路径片段指定长度比例处姿态
             */
            Eigen::Vector3d getConfigAngleAxis(double t) const;

            /// @brief 路径片段长度
            double length_;

            /// @brief 起点位姿
            Vector6d start_;
            /// @brief 终点位姿
            Vector6d end_;
            /// @brief 起点姿态，四元数形式
            Eigen::Quaterniond start_q_;
            /// @brief 终点姿态，四元数形式
            Eigen::Quaterniond end_q_;

            /// @brief 路径片段类型
            SegmentType type_ = SegmentType::UNKNOWN;
        };

        /// @brief 直线路径片段，继承自PathSegment类
        class LinearPathSegment : public PathSegment {
           public:
            /**
             * @brief 直线路径片段类构造函数
             * @param start 起点位姿
             * @param end 终点位姿
             */
            explicit LinearPathSegment(const Vector6d& start, const Vector6d& end);

            /**
             * @brief 获取路径片段距起点指定弧长处位姿
             * @param s 期望处位姿距起点弧长
             * @return std::vector<double> 路径片段距起点指定弧长处位姿
             */
            std::vector<double> getConfig(double s) const final;
        };

        /// @brief 直线圆弧路径片段，继承自PathSegment类
        class CircularPathSegment : public PathSegment {
           public:
            /**
            * @brief 圆弧路径片段类构造函数
            * @param start 起点位姿
            * @param end 终点位姿
            * @param center_pos 圆心位置
            * @param axis 旋转轴
            */
            explicit CircularPathSegment(const Vector6d& start, const Vector6d& end, const Eigen::Vector3d& center_pos,
                                         const Eigen::Vector3d& axis);

            /**
             * @brief 获取路径片段距起点指定弧长处位姿
             * @param s 期望处位姿距起点弧长
             * @return std::vector<double> 路径片段距起点指定弧长处位姿
             */
            std::vector<double> getConfig(double s) const final;

           private:
            /// @brief 圆弧对应圆心角
            double angle_;
            /// @brief 圆心指向起点向量
            Eigen::Vector3d center2start_;
            /// @brief 圆心位置
            Eigen::Vector3d center_pos_;
            /// @brief 旋转轴矢量
            Eigen::Vector3d axis_;
        };

        /// @brief 路径类
        class Path {
           public:
            Path()  = default;
            ~Path() = default;

            /**
             * @brief 添加路径片段
             * @param segment 路径片段指针
             */
            void addSegment(std::shared_ptr<PathSegment> segment);

            /**
             * @brief 获取路径距起点指定弧长处位姿
             * @param s 期望处位姿距起点弧长
             * @return std::vector<double> 路径距起点指定弧长处位姿
             */
            std::vector<double> getConfig(double s) const;
            /**
             * @brief 获取路径片段长度
             * @return double 路径片段长度
             */
            double getLength() const { return length_; };

            std::vector<std::vector<double>> getPathSegmentsSgPose();

            size_t getSegmentsNum() const { return segments_.size(); }

            double getLength(size_t id) const { return segments_[id]->getLength(); }

            std::vector<double> getConfig(size_t id, double s);

           private:
            /**
             * @brief 获取距路径起点指定弧长处路径片段索引
             * @param s 期望点至路径起点距离
             * @return int 距路径起点指定弧长处路径片段索引
             */
            int findSegmentsInd(double s) const;
            /**
             * @brief 检验两位姿是否连续（全等）
             * @param pose1 位姿1
             * @param pose2 位姿2
             * @param precision 判断精度
             * @return true 相等
             * @return false 不等
             */
            bool checkContinuous(const Vector6d& pose1, const Vector6d& pose2, double precision = 0.0001) const;

            /// @brief 各路径段起点处在路径上的弧长位置
            std::vector<double> start_lengths_ = {0};
            /// @brief 路径片段列表
            std::vector<std::shared_ptr<PathSegment>> segments_;

            /// @brief 路径长度
            double length_ = 0;
        };

        /**
         * @brief 读取多个示教点组成的多端直线轨迹（用多段直线代替曲线）
         * @param wps 
         * @param path 
         */
        void createPathWPs(std::vector<std::vector<double>> wps, Path& path);

        /**
         * @brief 读取直线+圆弧路径
         * @param config_file 配置文件（内含路径片段数据）
         * @param path 待写入路径
         */
        void readPathCicleLine(std::string config_file, Path& path);

        /**
         * @brief calculate circle from three points
         * https://stackoverflow.com/questions/13977354/build-circle-from-3-points-in-3d-space-implementation-in-c-or-c
         * @param p1 
         * @param p2 
         * @param p3 
         * @return std::pair<Eigen::Vector3d, double> 
         */
        std::pair<Eigen::Vector3d, double> CalCircleFromThreePoints(const Eigen::Vector3d& p1,
                                                                    const Eigen::Vector3d& p2,
                                                                    const Eigen::Vector3d& p3);
    }  // namespace tcl
}  // namespace vp::tp

#endif