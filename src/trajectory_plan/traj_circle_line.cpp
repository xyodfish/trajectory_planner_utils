/**
 * @file traj_circle_line.cpp
 * @author original.author@example.com
 * @brief 直线＋圆弧 轨迹规划算法，此文件涉及算法由TOTG算法path部分裁剪、修改后得到，
 * 将6D路径（直线、圆弧）按弧长平展至1维，随后使用梯形速度轨迹规划，路径中各点姿态由其
 * 对应路径段起终点姿态插值得到
 * @version 0.1
 * @date 2023-11-01
 *
 * @copyright Copyright (c) 2023 Original Source
 */

#include "traj_circle_line.h"
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>
#include "motionTransform.h"
#include "traj.h"

#include "load_yaml.h"

#ifdef USE_MSVC
#define _USE_MATH_DEFINES
#include <math.h>
#endif

namespace vp::tp {
    namespace tcl {
        std::vector<double> Vector6dToSTD(const Vector6d& vec) {
            std::vector<double> vec_std(vec.size());
            for (int i = 0; i < vec.size(); i++) {
                vec_std[i] = vec[i];
            }
            return vec_std;
        }

        void createPathWPs(std::vector<std::vector<double>> wps, Path& path) {
            for (int i = 0; i < wps.size() - 1; i++) {
                Vector6d start                            = Eigen::Map<Vector6d>(wps[i].data());
                Vector6d end                              = Eigen::Map<Vector6d>(wps[i + 1].data());
                std::shared_ptr<PathSegment> path_segment = std::make_shared<LinearPathSegment>(start, end);
                path.addSegment(path_segment);
            }
        };

        void readPathCicleLine(std::string config_file, Path& path) {
            YAML::Node yaml = YAML::LoadFile(config_file);
            YAML::Node yaml_node;
            if (yaml["segments"]) {
                yaml_node = yaml["segments"];
            } else if (yaml.IsSequence()) {
                yaml_node = yaml;
            } else {
                spdlog::error("invalid circle-line yaml format: {}", config_file);
                return;
            }
            for (int i = 0; i < yaml_node.size(); i++) {
                const auto& node = yaml_node[i];

                std::vector<double> start_std = load_yaml<std::vector<double>>(node, "start");
                std::vector<double> end_std   = load_yaml<std::vector<double>>(node, "end");
                std::string segment_type      = load_yaml<std::string>(node, "type");

                Vector6d start = Eigen::Map<Vector6d>(start_std.data());
                Vector6d end   = Eigen::Map<Vector6d>(end_std.data());

                if (segment_type == "linear") {
                    std::shared_ptr<PathSegment> path_segment = std::make_shared<LinearPathSegment>(start, end);
                    path.addSegment(path_segment);
                }
                // 圆弧路径片段可由两种方式给出：
                // 1.给定圆弧起点、终点、轴线矢量、半径，对应参数 type 应写为 circular_axis
                // 2.给定圆弧起点、终点、圆心位置，是否绕轴正转，对应参数 type 应写为 circular_axis
                else if (segment_type == "circular_axis") {
                    std::vector<double> axis_std = load_yaml<std::vector<double>>(node, "axis");
                    Eigen::Vector3d axis         = Eigen::Map<Eigen::Vector3d>(axis_std.data());

                    double radius = load_yaml<double>(node, "radius");

                    Eigen::Vector3d center;
                    {
                        Eigen::Quaterniond start_q = vp::math::getQuaternionFromTcpPose(start_std);
                        Eigen::Quaterniond end_q   = vp::math::getQuaternionFromTcpPose(end_std);
                        Eigen::AngleAxisd aa;
                        aa = end_q * start_q.inverse();

                        center =
                            vp::tp::calArcCenter(start.head(3), end.head(3), axis, radius, aa.angle() < M_PI);
                    }

                    std::shared_ptr<PathSegment> path_segment =
                        std::make_shared<CircularPathSegment>(start, end, center, axis);
                    path.addSegment(path_segment);
                } else if (segment_type == "circular_cen") {
                    std::vector<double> center_std = load_yaml<std::vector<double>>(node, "center");
                    Eigen::Vector3d center         = Eigen::Map<Eigen::Vector3d>(center_std.data());
                    bool positive                  = load_yaml<bool>(node, "positive");

                    Eigen::Vector3d axis = vp::tp::calArcAxis(start.head(3), end.head(3), center, positive);

                    std::shared_ptr<PathSegment> path_segment =
                        std::make_shared<CircularPathSegment>(start, end, center, axis);
                    path.addSegment(path_segment);
                } else if (segment_type == "circular_three_points") {
                    std::vector<double> mid_std = load_yaml<std::vector<double>>(node, "mid");
                    Vector6d mid                = Eigen::Map<Vector6d>(mid_std.data());

                    auto p                  = CalCircleFromThreePoints(start.head(3), mid.head(3), end.head(3));
                    Eigen::Vector3d center  = p.first;
                    double radius           = p.second;
                    Eigen::Vector3d mid_pos = mid.head(3);
                    Eigen::Vector3d axis =
                        vp::tp::calArcAxis(start.head(3), end.head(3), center, true, &mid_pos);

                    std::shared_ptr<PathSegment> path_segment =
                        std::make_shared<CircularPathSegment>(start, end, center, axis);
                    path.addSegment(path_segment);
                }
            }
        }

        Eigen::Vector3d PathSegment::getConfigAngleAxis(double t) const {
            Eigen::AngleAxisd config_aa_eigen = Eigen::AngleAxisd(start_q_.slerp(t, end_q_));
            return config_aa_eigen.angle() * config_aa_eigen.axis();
        }

        LinearPathSegment::LinearPathSegment(const Vector6d& start, const Vector6d& end) : PathSegment(start, end, 0) {
            Eigen::Vector3d start_pos = start.head(3);
            Eigen::Vector3d end_pos   = end.head(3);

            length_ = (start_pos - end_pos).norm();

            type_ = SegmentType::LINEAR;
        }

        std::vector<double> LinearPathSegment::getConfig(double s) const {
            s /= length_;
            s = std::max(0.0, std::min(1.0, s));

            Vector6d config = (1.0 - s) * start_ + s * end_;
            return Vector6dToSTD(config);
        }

        CircularPathSegment::CircularPathSegment(const Vector6d& start, const Vector6d& end,
                                                 const Eigen::Vector3d& center_pos, const Eigen::Vector3d& axis)
            : center_pos_(center_pos), axis_(axis), PathSegment(start, end, (end - start).norm()) {
            Eigen::Vector3d start_pos = start.head(3);

            Eigen::Vector3d end_pos = end.head(3);

            center2start_           = start_pos - center_pos;
            Eigen::Vector3d c2s_dir = center2start_.normalized();
            Eigen::Vector3d c2e_dir = (end_pos - center_pos).normalized();

            angle_ = std::acos(c2s_dir.dot(c2e_dir));

            if (axis.dot(c2s_dir.cross(c2e_dir)) < 0) {
                angle_ = 2 * M_PI - angle_;
            }

            double radius = center2start_.norm();

            length_ = radius * angle_;

            type_ = SegmentType::CIRCULAR;
        }

        std::vector<double> CircularPathSegment::getConfig(double s) const {
            s /= length_;
            s = std::max(0.0, std::min(1.0, s));

            Eigen::AngleAxisd aa(s * angle_, axis_);
            Eigen::Vector3d center2p   = aa.toRotationMatrix() * center2start_;
            Eigen::Vector3d config_pos = (aa.toRotationMatrix() * center2start_) + center_pos_;

            Eigen::Vector3d config_aa = getConfigAngleAxis(s);

            Vector6d config;

            config << config_pos[0], config_pos[1], config_pos[2], config_aa[0], config_aa[1], config_aa[2];

            return Vector6dToSTD(config);
        }

        void Path::addSegment(std::shared_ptr<PathSegment> segment) {
            if (!segments_.empty() && !checkContinuous(segments_.back()->getEnd(), segment->getStart())) {
                spdlog::warn("segment is not continuous");
            }

            segments_.push_back(segment);
            length_ += segment->getLength();
            start_lengths_.push_back(length_);
        }

        std::vector<double> Path::getConfig(double s) const {
            if (s < 0 || s > length_) {
                spdlog::error("should be between 0 and length");
                return {};
            }

            int ind = findSegmentsInd(s);
            return segments_[ind]->getConfig(s - start_lengths_[ind]);
        }

        std::vector<double> Path::getConfig(size_t id, double t) {

            if ((t > 1.0) && (t - 1.0) < 1e-5) {
                t = 1.0;
            }

            if (id < 0 || id > segments_.size() - 1) {
                spdlog::error("segement id  should be between 0 and max id");
                return {};
            }

            if (t < 0 || t > 1.0) {
                spdlog::error("s should be between 0 and 1 {}", t);
                return {};
            }

            return segments_[id]->getConfig(t * segments_[id]->getLength());
        }

        int Path::findSegmentsInd(double s) const {
            int ind = 0;
            while (s >= start_lengths_[ind + 1] && start_lengths_[ind + 1] != length_) {
                ind++;
            }

            return ind;
        }

        bool Path::checkContinuous(const Vector6d& pose1, const Vector6d& pose2, double precision) const {
            return pose1.isApprox(pose2, precision);
        }

        std::vector<std::vector<double>> Path::getPathSegmentsSgPose() {
            if (segments_.empty()) {
                return {};
            }

            std::vector<std::vector<double>> sg_vec;
            for (const auto& seg : segments_) {
                sg_vec.push_back(vp::math::vector6dToStdPose(seg->getStart()));
            }

            sg_vec.push_back(vp::math::vector6dToStdPose(segments_.back()->getEnd()));

            return sg_vec;
        }

        std::pair<Eigen::Vector3d, double> CalCircleFromThreePoints(const Eigen::Vector3d& p1,
                                                                    const Eigen::Vector3d& p2,
                                                                    const Eigen::Vector3d& p3) {
            Eigen::Vector3d t = p2 - p1;
            Eigen::Vector3d u = p3 - p1;
            Eigen::Vector3d v = p3 - p2;

            // triangle normal
            Eigen::Vector3d w = t.cross(u);
            const double wsl  = w.dot(w);

            // helpers
            const double iwsl2 = 1.0 / (2.0 * wsl);
            const double tt    = t.dot(t);
            const double uu    = u.dot(u);
            const double vv    = v.dot(v);

            // result circle
            Eigen::Vector3d circCenter = p1 + (u * tt * u.dot(v) - t * uu * t.dot(v)) * iwsl2;
            double circRadius          = std::sqrt(tt * uu * vv * iwsl2 * 0.5);
            Eigen::Vector3d circAxis   = w / sqrt(wsl);

            std::cout << "center: " << circCenter << "\n";
            std::cout << "radius: " << circRadius << "\n";

            return {circCenter, circRadius};
        }
    }  // namespace tcl
}  // namespace vp::tp
