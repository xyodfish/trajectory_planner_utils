#include "sas_segments.h"
#ifdef USE_MSVC
#define _USE_MATH_DEFINES
#include <math.h>
#endif

namespace vp::tp {
    namespace SAS {

        std::vector<double> SASegment::_getDesignatedPose(double length) {
            double t = std::max(0.0, std::min(1.0, length / length_));
            return _getInterpolationPose(t);
        }

        StraightLineSegment::StraightLineSegment(const Vector6d& start, const Vector6d& end) {
            m_start_ = vp::math::vector6dToStdPose(start);
            m_end_   = vp::math::vector6dToStdPose(end);

            length_  = (end - start).head(3).norm();
            segType_ = SegType::STRAIGHT;
        }

        StraightLineSegment::StraightLineSegment(const std::pair<Vector6d, Vector6d>& pts) {
            m_start_ = vp::math::vector6dToStdPose(pts.first);
            m_end_   = vp::math::vector6dToStdPose(pts.second);

            length_  = (pts.first - pts.second).head(3).norm();
            segType_ = SegType::STRAIGHT;
        }

        std::vector<double> StraightLineSegment::_getInterpolationPose(double t) {
            t = std::max(0.0, std::min(1.0, t));
            return vp::math::tcpPoseInterpolation(m_start_, m_end_, t);
        }

        ArcSegment::ArcSegment(const Vector6d& start, const Vector6d& goal, const Eigen::Vector3d& center,
                               const Eigen::Vector3d& planeNormal, double radius, double angle) {

            arcVal_.s_ = start;
            arcVal_.g_ = goal;
            arcVal_.c_ = center;
            arcVal_.n_ = planeNormal;
            arcVal_.r_ = radius;
            arcVal_.t_ = angle;

            Eigen::Vector3d U, V, W;
            U = (arcVal_.s_.head(3) - arcVal_.c_).normalized();
            W = arcVal_.n_.normalized();
            V = W.cross(U);

            R_ << U, V, W;

            qf_ = vp::math::getQuaternionFromTcpPose(vp::math::vector6dToStdPose(arcVal_.s_));
            qt_ = vp::math::getQuaternionFromTcpPose(vp::math::vector6dToStdPose(arcVal_.g_));

            length_ = arcVal_.r_ * arcVal_.t_;
        }

        ArcSegment::ArcSegment(const ArcValClass& arcVal) : arcVal_(arcVal) {

            Eigen::Vector3d U, V, W;
            U = (arcVal_.s_.head(3) - arcVal_.c_).normalized();
            W = arcVal_.n_.normalized();
            V = W.cross(U);

            R_ << U, V, W;

            qf_ = vp::math::getQuaternionFromTcpPose(vp::math::vector6dToStdPose(arcVal_.s_));
            qt_ = vp::math::getQuaternionFromTcpPose(vp::math::vector6dToStdPose(arcVal_.g_));

            length_ = arcVal_.r_ * arcVal_.t_;
        }

        std::vector<double> ArcSegment::_getInterpolationPose(double t) {
            t = std::max(0.0, std::min(1.0, t));

            Eigen::Vector3d point;

            point[0] = arcVal_.r_ * std::cos(t * arcVal_.t_);
            point[1] = arcVal_.r_ * std::sin(t * arcVal_.t_);
            point[2] = 0;

            Eigen::Vector3d pos = R_ * point + arcVal_.c_;

            Eigen::Quaterniond q = qf_.slerp(t, qt_);

            std::vector<double> ret_pose{pos[0], pos[1], pos[2], 0, 0, 0};

            vp::math::setTcpPoseFromQuaternion(ret_pose, q);
            return ret_pose;
        }

        PiecewiseSegments::PiecewiseSegments() {
            length_vec_.push_back(0.0);
        }

        double PiecewiseSegments::getSegLength() const {
            if (m_segments_.empty()) {
                return 0.0;
            }
            return length_vec_.back();
        }

        void PiecewiseSegments::appendSegment(const std::shared_ptr<SASegment> seg) {
            m_segments_.push_back(seg);
            length_vec_.push_back(length_vec_.back() + seg->getSegLength());
            length_ = length_vec_.back();
        }

        void PiecewiseSegments::clear() {
            m_segments_.clear();
        }

        bool PiecewiseSegments::empty() const {
            return m_segments_.empty();
        }

        size_t PiecewiseSegments::getSegmentsNum() const {
            return m_segments_.size();
        }

        std::vector<double> PiecewiseSegments::_getDesignatedPose(double length) {
            auto it_lower = std::lower_bound(length_vec_.begin(), length_vec_.end(), length);

            // 如果找到的元素在 length_vec_ 的开始位置，直接调用第一个 segment 的方法
            if (it_lower == length_vec_.begin()) {
                return m_segments_[0]->_getDesignatedPose(length);
            }
            // 如果找不到满足条件的元素，即它_lower等于 length_vec_ 的 end()，
            // 则调用最后一个 segment 的方法，并从 length 中减去前一个元素的值
            else if (it_lower == length_vec_.end()) {
                return m_segments_.back()->_getDesignatedPose(length - length_vec_[length_vec_.size() - 2]);
            }
            // 如果找到了满足条件的元素，则用当前给定的length 减去 lengh_vec中上个一个元素的length 则得到当前给定length
            // 在其既有所属轨迹段中的所代表的长度
            else {
                auto index = std::distance(length_vec_.begin(), it_lower);
                return m_segments_[index - 1]->_getDesignatedPose(length - length_vec_[index - 1]);
            }
        }

        std::vector<double> PiecewiseSegments::_getInterpolationPose(double t) {
            double cur_length = t * length_vec_.back();
            return _getDesignatedPose(cur_length);
        }

        SegType PiecewiseSegments::_getSegType(double t) {

            auto it_lower = std::lower_bound(length_vec_.begin(), length_vec_.end(), t * length_vec_.back());

            // 如果找到的元素在 length_vec_ 的开始位置，直接调用第一个 segment 的方法
            if (it_lower == length_vec_.begin()) {
                return m_segments_[0]->getSegType();
            }
            // 如果找不到满足条件的元素，即它_lower等于 length_vec_ 的 end()，
            // 则调用最后一个 segment 的方法，并从 length 中减去前一个元素的值
            else if (it_lower == length_vec_.end()) {
                return m_segments_.back()->getSegType();
            }
            // 如果找到了满足条件的元素，则用当前给定的length 减去 lengh_vec中上个一个元素的length 则得到当前给定length
            // 在其既有所属轨迹段中的所代表的长度
            else {
                auto index = std::distance(length_vec_.begin(), it_lower);
                return m_segments_[index - 1]->getSegType();
            }
        }

        size_t PiecewiseSegments::_getSegId(double t) {
            auto it_lower = std::lower_bound(length_vec_.begin(), length_vec_.end(), t * length_vec_.back());
            // 如果找到的元素在 length_vec_ 的开始位置，直接调用第一个 segment 的方法
            if (it_lower == length_vec_.begin()) {
                return 1;
            }
            // 如果找不到满足条件的元素，即它_lower等于 length_vec_ 的 end()，
            // 则调用最后一个 segment 的方法，并从 length 中减去前一个元素的值
            else if (it_lower == length_vec_.end()) {
                return length_vec_.size();
            }
            // 如果找到了满足条件的元素，则用当前给定的length 减去 lengh_vec中上个一个元素的length 则得到当前给定length
            // 在其既有所属轨迹段中的所代表的长度
            else {
                auto index = std::distance(length_vec_.begin(), it_lower);
                return index;
            }
        }

        std::pair<double, double> PiecewiseSegments::_getSegLengthPeriod(size_t id) {
            if (id < 1 || id > length_vec_.size()) {
                return {};
            }

            return {length_vec_[id - 1] / length_, length_vec_[id] / length_};
        }

        std::pair<size_t, SegType> PiecewiseSegments::_getSegInfo(double t) {
            return {_getSegId(t), _getSegType(t)};
        }

    }  // namespace SAS

    /// @brief 三点内切园求解
    void InscribedCircleArc::setBoundaries(const Vector6d& p1, const Vector6d& p2, const Vector6d& p3,
                                           double contact_dis) {
        p1_          = p1;
        p2_          = p2;
        p3_          = p3;
        contact_dis_ = contact_dis;

        v1_ = (p1_ - p2_).head(3);
        v2_ = (p3_ - p2_).head(3);
        v3_ = (0.5 * (v1_.normalized() + v2_.normalized())).normalized();

        q1_ = vp::math::getQuaternionFromTcpPose(p1_);
        q2_ = vp::math::getQuaternionFromTcpPose(p2_);
        q3_ = vp::math::getQuaternionFromTcpPose(p3_);

        solve();
    }

    void InscribedCircleArc::solve() {
        solveRadius();
        solveCenter();
        solveArcRad();
        solveCPts();
        solveMidPt();
        solvePlaneNormal();
    }

    void InscribedCircleArc::solveRadius() {
        vecAngle_ = 0.5 * vp::math::getTwoVectorAngles(v1_, v2_);
        radius_   = contact_dis_ * std::tan(vecAngle_);
    }

    void InscribedCircleArc::solveCenter() {
        auto dis = contact_dis_ / std::cos(vecAngle_);
        center_  = p2_.head(3) + dis * v3_;
    }

    void InscribedCircleArc::solveArcRad() {
        arcRad_ = M_PI - 2 * vecAngle_;
    }

    void InscribedCircleArc::solveCPts() {
        auto con1_coeff = contact_dis_ / v1_.norm();  // 切点1到在起点与中间过渡点中的比例关系
        auto con2_coeff = contact_dis_ / v2_.norm();  // 切点2到在中间过渡点与终点中的比例关系

        lcpt_ = vp::math::vector6dPoseInterpolation(p2_, p1_, con1_coeff);
        rcpt_ = vp::math::vector6dPoseInterpolation(p2_, p3_, con2_coeff);
    }

    void InscribedCircleArc::solveMidPt() {
        arcMidPt_.head(3) = center_ - v3_ * radius_;
        vp::math::setVector6dPoseFromQuaternion(arcMidPt_, q2_);
    }

    void InscribedCircleArc::solvePlaneNormal() {
        Eigen::Vector3d lpt = lcpt_.head(3);
        Eigen::Vector3d rpt = rcpt_.head(3);

        planeNormal_ = (lpt - center_).cross(rpt - center_).normalized();
    }

    SAS::ArcValClass InscribedCircleArc::getLArcVal() {

        return {lcpt_, arcMidPt_, center_, planeNormal_, radius_, 0.5 * arcRad_};
    }

    SAS::ArcValClass InscribedCircleArc::getRArcVal() {
        return {arcMidPt_, rcpt_, center_, planeNormal_, radius_, 0.5 * arcRad_};
    }

    SAS::ArcValClass InscribedCircleArc::getArcVal() {
        return {lcpt_, rcpt_, center_, planeNormal_, radius_, arcRad_};
    }

}  // namespace vp::tp