#include "sasTimeUniformTraj.h"
#include "multiPointsTrajectory.h"
#include "traj.h"

namespace vp::tp {

    MultiPointsTrajectory::SASTimeUniformTraj::SASTimeUniformTraj(const std::vector<std::vector<double>>& m_points,
                                                                  const double& cont_dis_coeff)
        : m_points_(m_points), cont_dis_coeff_(cont_dis_coeff) {}

    double MultiPointsTrajectory::SASTimeUniformTraj::calculateCutInDistance(double cut_in_coeff) {

        double min_distance = std::numeric_limits<double>::max();

        for (auto cur_id = 0; cur_id < m_points_.size() - 1; ++cur_id) {
            auto distance = vp::math::twoVectorDistance(m_points_[cur_id], m_points_[cur_id + 1]);
            if (distance < min_distance) {
                min_distance = distance;
            }
        }

        return cut_in_coeff * min_distance;
    }

    void MultiPointsTrajectory::SASTimeUniformTraj::addPiecewiseSegments(
        const std::vector<std::pair<Vector6d, Vector6d>>& sl_pts, const std::vector<SAS::ArcValClass>& arc_pts) {
        int si_ = 0, ai_ = 0;
        seg_sptr[0] = std::make_shared<SAS::StraightLineSegment>(sl_pts[0]);
        seg_sptr[1] = std::make_shared<SAS::ArcSegment>(arc_pts[0]);
        ai_++;
        si_++;

        for (int cur_id = 2; cur_id < segNum_ - 2; cur_id = cur_id + 3) {
            // 通过起手式后 后续为A->l->A的顺序
            seg_sptr[cur_id]     = std::make_shared<SAS::ArcSegment>(arc_pts[ai_]);
            seg_sptr[cur_id + 1] = std::make_shared<SAS::StraightLineSegment>(sl_pts[si_]);
            seg_sptr[cur_id + 2] = std::make_shared<SAS::ArcSegment>(arc_pts[ai_ + 1]);

            ai_ = ai_ + 2;
            si_++;
        }

        seg_sptr[segNum_ - 2] = std::make_shared<SAS::ArcSegment>(arc_pts[ai_]);
        seg_sptr[segNum_ - 1] = std::make_shared<SAS::StraightLineSegment>(sl_pts[si_]);

        for (auto& ptr : seg_sptr) {
            ps_sptr->appendSegment(ptr);
        }
    }

    void MultiPointsTrajectory::SASTimeUniformTraj::solveSASValue(double cut_in_dis) {

        InscribedCircleArc ica;
        std::vector<SAS::ArcValClass> arc_pts;
        std::vector<std::pair<Vector6d, Vector6d>> sl_pts;
        std::pair<Vector6d, Vector6d> sl_pt;

        sl_pt.first = m_pts_[0];

        for (int cur_id = 0; cur_id < piecewiseNum_; ++cur_id) {

            // set ica problem boundary
            ica.setBoundaries(m_pts_[cur_id], m_pts_[cur_id + 1], m_pts_[cur_id + 2], cut_in_dis);

            // add left arc clas and right arc class
            arc_pts.push_back(ica.getLArcVal());
            arc_pts.push_back(ica.getRArcVal());

            sl_pt.second = ica.getLCPt();
            sl_pts.push_back(sl_pt);
            sl_pt.first = ica.getRCPt();

            blendRadius_[cur_id] = ica.getRadius();

            if (cur_id + 1 == piecewiseNum_) {
                sl_pt.second = m_pts_.back();
                sl_pts.push_back(sl_pt);
            }
        }

        addPiecewiseSegments(sl_pts, arc_pts);
    }

    void MultiPointsTrajectory::SASTimeUniformTraj::initialization() {
        init_params();
        auto min_distance = calculateCutInDistance(cont_dis_coeff_);
        solveSASValue(min_distance);
    }

    void MultiPointsTrajectory::SASTimeUniformTraj::init_params() {

        m_pts_.clear();
        for (auto& pt : m_points_) {
            m_pts_.push_back(vp::math::vectorToEigen(pt));
        }

        ps_sptr = std::make_shared<SAS::PiecewiseSegments>();

        segNum_ = 4 + 3 * (m_pts_.size() - 3);
        seg_sptr.resize(segNum_);
        piecewiseNum_ = m_pts_.size() - 2;

        blendRadius_.resize(piecewiseNum_);
    }

    std::shared_ptr<SAS::PiecewiseSegments>& MultiPointsTrajectory::SASTimeUniformTraj::getPiecewiseSegments() {
        return ps_sptr;
    }

    std::shared_ptr<VelocityPlannerCompat>& MultiPointsTrajectory::SASTimeUniformTraj::getVelPlanner() {
        return vel_planner;
    }

    std::vector<double> MultiPointsTrajectory::SASTimeUniformTraj::getBlendRadius() const {
        return blendRadius_;
    }

    double MultiPointsTrajectory::SASTimeUniformTraj::getSegLength() const {
        return ps_sptr->getSegLength();
    }

    void MultiPointsTrajectory::SASTimeUniformTraj::setWayPoints(const std::vector<std::vector<double>>& points) {
        m_points_ = points;
    }

    std::vector<std::vector<double>> MultiPointsTrajectory::SASTimeUniformTraj::getTrajs() {

        auto data = vel_planner->getTrajs(true);
        auto num  = data.size();

        std::vector<std::vector<double>> val;

        double t  = 0.0;
        double dt = data[1][time_index_] - data[0][time_index_];

        for (auto i = 0; i < num; ++i) {
            // 直线段1
            t = data[i][pose_index_];
            appendTrajData(t, val, data[i][time_index_], dt, i);
        }

        return val;
    }

    std::vector<double> MultiPointsTrajectory::SASTimeUniformTraj::getInterpolationPose(double t) {
        return ps_sptr->_getInterpolationPose(t);
    }

    void MultiPointsTrajectory::SASTimeUniformTraj::calculateVelAndAcc(std::vector<std::vector<double>>& val, size_t id,
                                                                       double dt) {
        /// @brief calculate vel
        if (id >= 1) {
            for (int j = 0; j < 6; ++j) {
                val[id][j + vel_index_] = (val[id][j + pose_index_] - val[id - 1][j + pose_index_]) / dt;

                val[id][j + acc_index_] = (val[id][j + vel_index_] - val[id - 1][j + vel_index_]) / dt;
            }

        } else {
            for (int j = 0; j < 6; ++j) {
                val[id][j + vel_index_] = 0;
                val[id][j + acc_index_] = 0;
            }
        }
    }

    void MultiPointsTrajectory::SASTimeUniformTraj::appendTrajData(double t, std::vector<std::vector<double>>& val,
                                                                   double time, double dt, double id) {
        static std::vector<double> tp(6 * 3 + 1, 0);
        static std::vector<double> pose(6);

        tp[time_index_] = time;
        pose            = ps_sptr->_getInterpolationPose(t);

        std::copy(pose.begin(), pose.end(), tp.begin() + 1);
        val.push_back(tp);

        calculateVelAndAcc(val, id, dt);
    }

    void MultiPointsTrajectory::SASTimeUniformTraj::setVelPlanner(const std::vector<BCs<double>>& BCs,
                                                                  const std::string& algo) {

        m_BCs       = BCs;
        m_alg       = algo;
        vel_planner = std::make_shared<VelocityPlannerCompat>(BCs, algo);
    }

    void MultiPointsTrajectory::SASTimeUniformTraj::setVelPlanner(const std::vector<BCs<double>>& BCs,
                                                                  const std::string& algo,
                                                                  const std::shared_ptr<Vpb3d>& params) {
        m_BCs = BCs;
        m_alg = algo;

        if (params) {
            vel_planner = std::make_shared<VelocityPlannerCompat>(BCs, algo, params);
        } else {
            vel_planner = std::make_shared<VelocityPlannerCompat>(BCs, algo);
        }
    }

    void MultiPointsTrajectory::SASTimeUniformTraj::setVelPlanner(const std::vector<BCs<double>>& BCs) {

        m_BCs       = BCs;
        vel_planner = std::make_shared<VelocityPlannerCompat>(BCs, m_alg);
    }

    std::vector<std::vector<double>> MultiPointsTrajectory::SASTimeUniformTraj::getRawTrajs() {
        return vel_planner->getTrajs(true);
    }

}  // namespace vp::tp