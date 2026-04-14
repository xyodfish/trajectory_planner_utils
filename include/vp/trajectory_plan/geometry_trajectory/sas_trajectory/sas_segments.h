#ifndef __SAS_FUNCTIONS_H__
#define __SAS_FUNCTIONS_H__

#include "Eigen/Core"
#include "Eigen/Dense"
#include "motionTransform.h"

namespace vp::tp {

    namespace SAS {

        using Vector6d = Eigen::Matrix<double, 6, 1>;

        enum class SegType { UNKNOWN = 0, STRAIGHT, ARC };

        class ArcValClass {
           public:
            Vector6d s_;  // arc start
            Vector6d g_;  // arc goal

            Eigen::Vector3d c_;  // arc center
            Eigen::Vector3d n_;  // arc planeNormal

            double r_;  // arc radius
            double t_;  // arc theta
        };

        /**
         * @brief linear arc segment class
         * 
         */
        class SASegment {

           public:
            explicit SASegment() = default;
            virtual ~SASegment() = default;

            virtual double getSegLength() const { return length_; };

            virtual std::vector<double> _getInterpolationPose(double t) = 0;
            virtual std::vector<double> _getDesignatedPose(double length);
            virtual SegType getSegType() const { return SegType::UNKNOWN; }

           protected:
            std::vector<double> m_start_, m_end_;
            double length_;
            SegType segType_;
        };

        /**
         * @brief straight line segment
         * 
         */
        class StraightLineSegment : public SASegment {
           public:
            explicit StraightLineSegment(const Vector6d& start, const Vector6d& end);
            explicit StraightLineSegment(const std::pair<Vector6d, Vector6d>& pts);

            virtual std::vector<double> _getInterpolationPose(double t) override;

            SegType getSegType() const override { return SegType::STRAIGHT; }
        };

        class ArcSegment : public SASegment {

           public:
            explicit ArcSegment(const Vector6d& start, const Vector6d& end, const Eigen::Vector3d& center,
                                const Eigen::Vector3d& planeNormal, double radius, double angle);
            explicit ArcSegment(const ArcValClass& arcVal);

            virtual std::vector<double> _getInterpolationPose(double t) override;

            SegType getSegType() const override { return SegType::ARC; }

           private:
            ArcValClass arcVal_;
            Eigen::Matrix3d R_;

            Eigen::Quaterniond qf_, qt_;
        };

        class PiecewiseSegments : public SASegment {
           public:
            explicit PiecewiseSegments();
            void appendSegment(const std::shared_ptr<SASegment> seg);

            virtual std::vector<double> _getInterpolationPose(double t) override;
            virtual std::vector<double> _getDesignatedPose(double length) override;

            size_t getSegmentsNum() const;

            void clear();

            bool empty() const;

            virtual double getSegLength() const override;

            SegType _getSegType(double t);

            size_t _getSegId(double t);

            std::pair<double, double> _getSegLengthPeriod(size_t id);

            std::pair<size_t, SegType> _getSegInfo(double t);

           private:
            std::vector<std::shared_ptr<SASegment>> m_segments_;
            std::vector<double> length_vec_;
        };

    }  // namespace SAS

    using Vector6d = SAS::Vector6d;

    // 给点三点计算内切圆
    class InscribedCircleArc {

       public:
        explicit InscribedCircleArc() = default;

        void setBoundaries(const Vector6d& p1, const Vector6d& p2, const Vector6d& p3, double contact_dis);

        SAS::ArcValClass getLArcVal();
        SAS::ArcValClass getRArcVal();
        SAS::ArcValClass getArcVal();

        void solve();

        std::pair<Vector6d, Vector6d> getContactPt() const { return std::pair<Vector6d, Vector6d>{lcpt_, rcpt_}; }

        Vector6d& getLCPt() { return lcpt_; }
        Vector6d& getRCPt() { return rcpt_; }

        double getRadius() const { return radius_; }

       private:
        void solveRadius();
        void solveArcRad();

        void solveCenter();

        void solveMidPt();
        void solveCPts();

        void solvePlaneNormal();

        Vector6d p1_, p2_, p3_;
        Eigen::Vector3d v1_, v2_, v3_;
        double contact_dis_;
        double radius_;
        double vecAngle_;

        Eigen::Vector3d center_;

        Vector6d arcMidPt_;

        Eigen::Quaterniond q1_, q2_, q3_;

        Eigen::Vector3d planeNormal_;

        Vector6d lcpt_, rcpt_;

        double arcRad_;
    };

}  // namespace vp::tp
#endif
