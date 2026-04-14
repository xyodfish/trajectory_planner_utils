#include "sasTrajectory.h"
#include "dbg.h"
#include "motionTransform.h"
#include "spdlog/spdlog.h"
#include "traj.h"

#ifdef USE_MSVC
#define _USE_MATH_DEFINES
#include <math.h>
#endif

namespace vp::tp {
    using Vector6d = Eigen::Matrix<double, 6, 1>;

    SASTrajectory::SASTrajectory(const std::vector<double>& s_pose, const std::vector<double>& m_pose,
                                 const std::vector<double>& g_pose, std::vector<BCs<double>> BCs_,
                                 const std::string& alg, double contact_dis_coeff)
        : p1_(vp::math::vectorToEigen(s_pose)),
          p2_(vp::math::vectorToEigen(m_pose)),
          p3_(vp::math::vectorToEigen(g_pose)),
          m_BCs_(BCs_),
          m_alg(alg),
          contact_dis_coeff_(contact_dis_coeff) {
        setPlanner(BCs_, alg);
        m_plan_type = GIVEN_POINTS;
    }

    SASTrajectory::SASTrajectory(const std::vector<double>& s_pose, const std::vector<double>& m_pose,
                                 const std::vector<double>& g_pose, double contact_dis_coeff)
        : p1_(vp::math::vectorToEigen(s_pose)),
          p2_(vp::math::vectorToEigen(m_pose)),
          p3_(vp::math::vectorToEigen(g_pose)),
          contact_dis_coeff_(contact_dis_coeff) {
        initialization();
        m_plan_type = GIVEN_POINTS;
    }

    void SASTrajectory::initialization() {

        ps_sptr = std::make_shared<SAS::PiecewiseSegments>();

        auto con_dis1 = (p2_ - p1_).head(3).norm();
        auto con_dis2 = (p3_ - p2_).head(3).norm();

        m_con_dis_ = contact_dis_coeff_ * std::min(con_dis1, con_dis2);

        v1_ = (p1_ - p2_).head(3);
        v2_ = (p3_ - p2_).head(3);
        v3_ = (0.5 * (v1_.normalized() + v2_.normalized())).normalized();

        m_angle_   = vp::math::getTwoVectorAngles(v1_, v2_);
        m_angle_2_ = 0.5 * m_angle_;
        m_radius_  = m_con_dis_ * std::tan(m_angle_2_);

        auto dis = m_con_dis_ / std::cos(m_angle_2_);

        m_center_ = p2_.head(3) + dis * v3_;
        m_con_p1_ = p2_.head(3) + v1_.normalized() * m_con_dis_;
        m_con_p2_ = p2_.head(3) + v2_.normalized() * m_con_dis_;

        m_rad_     = M_PI - m_angle_;
        m_arc_dis_ = m_radius_ * m_rad_;

        auto contact_dis1 = (m_con_p1_ - p1_.head(3)).norm();
        auto contact_dis2 = (m_con_p2_ - p3_.head(3)).norm();

        m_planeNormal_ = (m_con_p1_ - m_center_).cross(m_con_p2_ - m_center_).normalized();

        m_traj_dis_ = (m_con_p1_ - p1_.head(3)).norm() + (m_con_p2_ - p3_.head(3)).norm() + m_arc_dis_;

        m_dis_coeff_[0] = contact_dis1 / m_traj_dis_;
        m_dis_coeff_[1] = m_arc_dis_ / m_traj_dis_;
        m_dis_coeff_[2] = contact_dis2 / m_traj_dis_;

        // cal contact pose [x, y, z, aax, aay, aaz]
        m_con1_coeff_ = m_con_dis_ / v1_.norm();  // 切点1到在起点与中间过渡点中的比例关系
        m_con2_coeff_ = m_con_dis_ / v2_.norm();  // 切点2到在中间过渡点与终点中的比例关系

        p_con1_ = vp::math::vector6dPoseInterpolation(p2_, p1_, m_con1_coeff_);
        p_con2_ = vp::math::vector6dPoseInterpolation(p2_, p3_, m_con2_coeff_);

        Eigen::Vector3d U, V, W;
        U = (m_con_p1_ - m_center_).normalized();
        W = m_planeNormal_;
        V = W.cross(U).normalized();

        R_ << U, V, W;

        q1_ = vp::math::getQuaternionFromTcpPose(p1_);
        q2_ = vp::math::getQuaternionFromTcpPose(p2_);
        q3_ = vp::math::getQuaternionFromTcpPose(p3_);

        Vector6d m_arc_middle_;
        m_arc_middle_.head(3) = m_center_ - v3_ * m_radius_;
        vp::math::setVector6dPoseFromQuaternion(m_arc_middle_, q2_);

        std::pair<Vector6d, Vector6d> sl1_sp_, sl2_sp_;
        std::pair<Vector6d, Vector6d> as1_sp_, as2_sp_;

        sl1_sp_.first  = p1_;
        sl1_sp_.second = vp::math::vector6dPoseInterpolation(p2_, p1_, m_con1_coeff_);
        sl2_sp_.first  = vp::math::vector6dPoseInterpolation(p2_, p3_, m_con2_coeff_);
        sl2_sp_.second = p3_;

        as1_sp_.first  = sl1_sp_.second;
        as1_sp_.second = m_arc_middle_;
        as2_sp_.first  = m_arc_middle_;
        as2_sp_.second = sl2_sp_.first;

        seg_sptr.resize(4);

        seg_sptr[0] = std::make_shared<SAS::StraightLineSegment>(sl1_sp_.first, sl1_sp_.second);
        seg_sptr[1] = std::make_shared<SAS::ArcSegment>(as1_sp_.first, as1_sp_.second, m_center_, m_planeNormal_,
                                                        m_radius_, 0.5 * m_rad_);

        seg_sptr[2] = std::make_shared<SAS::ArcSegment>(as2_sp_.first, as2_sp_.second, m_center_, m_planeNormal_,
                                                        m_radius_, 0.5 * m_rad_);

        seg_sptr[3] = std::make_shared<SAS::StraightLineSegment>(sl2_sp_.first, sl2_sp_.second);

        for (auto& ptr : seg_sptr) {
            ps_sptr->appendSegment(ptr);
        }
    }

    void SASTrajectory::setPlanner(const std::vector<double>& s_pose, const std::vector<double>& m_pose,
                                   const std::vector<double>& g_pose, double contact_dis_coeff) {
        p1_ = vp::math::vectorToEigen(s_pose), p2_ = vp::math::vectorToEigen(m_pose), p3_ = vp::math::vectorToEigen(g_pose);
        initialization();
        m_plan_type           = GIVEN_POINTS;
        m_BCs_[0].g_state.pos = ps_sptr->getSegLength();
        vel_planner           = std::make_shared<VelocityPlannerCompat>(m_BCs_, m_alg);
    }

    void SASTrajectory::setPlanner(const std::vector<BCs<double>>& BCs_, const std::string& alg) {
        if (((m_plan_type == GIVEN_MULTI_BCS) && (BCs_.size() != PLANNER_DIM)) ||
            ((m_plan_type == GIVEN_POINTS) && (BCs_.size() != 1))) {
            spdlog::error("input param size error!  input boundary conditions size {} is not match the plan type {}",
                          BCs_.size(), (int)m_plan_type);
            return;
        }

        initialization();

        auto BCs           = BCs_;
        BCs[0].g_state.pos = ps_sptr->getSegLength();
        vel_planner        = std::make_shared<VelocityPlannerCompat>(BCs, alg);
    }

    Vector6d SASTrajectory::caculatePoseOnArc(double t) {
        // 计算走过的角度
        Eigen::Vector3d point, arc_pos;
        Eigen::Quaterniond q;

        auto angle = ((t - m_dis_coeff_[0]) / m_dis_coeff_[1]) * m_rad_;
        point(0)   = m_radius_ * std::cos(angle);
        point(1)   = m_radius_ * std::sin(angle);
        point(2)   = 0;
        arc_pos    = R_ * point + m_center_;

        // 轨迹前半段
        auto beta = (t - m_dis_coeff_[0]) / m_dis_coeff_[1];
        if (beta < 0.5) {
            q = q1_.slerp(m_con1_coeff_ + 2 * beta * (1.0 - m_con1_coeff_), q2_);
        } else {
            q = q2_.slerp(beta - 0.5, q3_);
        }

        std::vector<double> ret_pose{arc_pos[0], arc_pos[1], arc_pos[2], 0.0, 0.0, 0.0};
        vp::math::setTcpPoseFromQuaternion(ret_pose, q);

        return vp::math::vectorToEigen(ret_pose);
    }

    std::vector<std::vector<KState<double>>> SASTrajectory::getKStates() const {
        return vel_planner->getKStates();
    }

    std::vector<std::vector<double>> SASTrajectory::getRawTrajs() {
        return vel_planner->getTrajs(true);
    }

    std::vector<std::vector<double>> SASTrajectory::getTrajs() {
        auto data = vel_planner->getTrajs(true);
        auto num  = data.size();

        std::vector<std::vector<double>> val(num, std::vector<double>(PLANNER_DIM * 3 + 1));
        std::vector<double> pose;

        Eigen::Vector3d arc_pos, point;
        double t = 0.0;

        for (int i = 0; i < num; ++i) {

            t         = data[i][pose_index_];
            val[i][0] = data[i][time_index_];
            pose      = ps_sptr->_getInterpolationPose(t);
            std::copy(pose.begin(), pose.end(), val[i].begin() + 1);

            if (i >= 1) {
                for (int j = 0; j < PLANNER_DIM; ++j) {
                    val[i][j + vel_index_] = (val[i][j + pose_index_] - val[i - 1][j + pose_index_]) /
                                             (val[i][time_index_] - val[i - 1][time_index_]);
                }
            }
        }

        // cal vel
        return val;
    }

}  // namespace vp::tp
