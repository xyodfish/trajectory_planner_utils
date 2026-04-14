/*
 * Notation:
 *      Let's define 2 coordinate frames - {Ref} and {Obj}.
 *      A rotation matrix rotation{Rel}{Obj} (variable name: rotationRelObj)
 * denotes the orientation of {Obj} relative to {Rel}. In other words, it
 * represent a rotation from {Obj} to {Ref}. The following properties holds:
 *                  rotation{A}{B} = inverse of rotation{B}{A} = transpose of
 * rotation{B}{A} rotation{A}{C} = rotation{A}{B} * rotation{B}{C} p{A} =
 * rotation{A}{B}*p{B}
 *
 *      A rotation can be defined by a rotation angle around an arbitrary axis.
 *                  rotation{Ref}{Obj} = rotation(axis, angle)
 *      which denotes that {Obj} is obtained by rotating {Ref} about (axis) by
 * (angle).
 *
 *      A vector p{Ref}{Obj} (variable name: pRelObj) denotes a vector from the
 * origin of {Obj} in {Ref}.
 *
 *      A homogeneous transformation matrix transformation{Ref}{Obj} (variable
 * name: frameRelObj) denotes a transformation from {Obj} to {Ref}, which is
 * defined as: transformation{Ref}{Obj} = transformation(rotation{Ref}{Obj},
 * p{Ref}{Obj}) = [rotation{Ref}{Obj}, p{Ref}{Obj}; 0,           1] For three
 * frames {A}, {B} and {C}, a straightforward calculation holds:
 *                  transformation{A}{B} = inverse of transformation{B}{A}
 *                  inverse of transformation{A}{B} =
 * [transpose(rotation{A}{B}), -transpose(rotation{A}{B})*p{A}{B}; 0, 1]
 *                  transformation{A}{C} = transformation{A}{B} *
 * transformation{B}{C} A Twist is a combination of linear and angular
 * velocities of a moving actualFrameBaseEE. It is defined as twist = [linear
 * velocity; angular velocity]; A Wrench is a combination of force and moment
 * applying on a moving FrameBaseEE. It is defined as wrench = [force; moment];
 *      In our case, the wrench is applied by the environment on the robot.
 *      A skew-symmetric matrix [p] of a vector p is defined as:
 *                  [p] = [    0 -p(2)  p(1);
 *                          p(2)    0 -p(0);
 *                         -p(1) p(0)    0]
 *      An adjoint matrix of transformation{Ref}{Obj} is defined as:
 *                  adjoint{Ref}{Obj} = [rotation{Ref}{Obj},
 * [p{Ref}{Obj}]*rotation{Ref}{Obj}; 0,               rotation{Ref}{Obj}] The
 * transformation of the twist from one frame to another can be expressed as:
 *                  twist{Ref} = adjoint{Ref}{Obj}*twist{Obj}
 *      The transformation of the wrench from one frame to another can be
 * expressed as: wrench{Obj} = adjoint{Ref}{Obj}^T*wrench{Ref}
 *
 *      Reference:
 *          "Modern Robotics: Mechanics, Planning, and Control," Kevin M. Lynch
 * and Frank C. Park, Cambridge University Press, 2017, ISBN 9781107156302
 */

#ifndef RIGID_BODY_DYNAMICS_H
#define RIGID_BODY_DYNAMICS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <iostream>
#include <vector>

#include "utility.h"

namespace vp::math {
    extern std::array<double, 3> g;
    extern std::array<double, 16> frameIdentity;
    extern std::array<double, 9> rotIdentity;

    enum FRAME { BASE, END_EFFECTOR };

    inline Eigen::Matrix3d get_rot(const Eigen::Matrix4d& frame);
    inline std::array<double, 9> get_rot(const std::array<double, 16>& frame);

    inline Eigen::Vector3d get_p(const Eigen::Matrix4d& frame);
    inline std::array<double, 3> get_p(const std::array<double, 16>& frame);

    inline Eigen::Matrix4d get_frame(const Eigen::Vector3d& p, const Eigen::Matrix3d& rot);
    inline std::array<double, 16> get_frame(const std::array<double, 3>& p, const std::array<double, 9>& rot);

    inline Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& p);
    inline std::array<double, 9> skew_symmetric(const std::array<double, 3>& vec);

    inline Eigen::Matrix3d rot_inverse(const Eigen::Matrix3d& rot);
    inline std::array<double, 9> rot_inverse(const std::array<double, 9>& rot);

    inline Eigen::Matrix4d frame_inverse(const Eigen::Matrix4d& frame);
    inline std::array<double, 16> frame_inverse(const std::array<double, 16>& frame);

    inline Eigen::Vector3d rotate(const Eigen::Matrix3d& rot, const Eigen::Vector3d& vec);
    inline std::array<double, 3> rotate(const std::array<double, 9>& rot, const std::array<double, 3>& vec);

    inline Eigen::Matrix<double, 6, 1> rotate(const Eigen::Matrix3d& rot, const Eigen::Matrix<double, 6, 1>& vec);
    inline std::array<double, 6> rotate(const std::array<double, 9>& rot, const std::array<double, 6>& vec);

    inline Eigen::Matrix4d frame_subtract(const Eigen::Matrix4d& frameRef, const Eigen::Matrix4d& frameObj);
    inline std::array<double, 16> frame_subtract(const std::array<double, 16>& frameRef,
                                                 const std::array<double, 16>& frameObj);

    inline std::pair<double, Eigen::Vector3d> get_angle_axis(const Eigen::Matrix3d& rot, double eps = epsilon);
    inline std::pair<double, std::array<double, 3>> get_angle_axis(const std::array<double, 9>& rot,
                                                                   double eps = epsilon);

    inline Eigen::Matrix3d get_rot(const double& angle, const Eigen::Vector3d& axis, double eps = epsilon);
    inline std::array<double, 9> get_rot(const double& angle, const std::array<double, 3>& axis, double eps = epsilon);

    inline Eigen::Matrix<double, 6, 1> frame2pose(const Eigen::Matrix4d& frame);
    inline std::array<double, 6> frame2pose(const std::array<double, 16>& frame);

    inline Eigen::Matrix4d pose2frame(const Eigen::Matrix<double, 6, 1>& pose);
    inline std::array<double, 16> pose2frame(const std::array<double, 6>& pose);

    inline Eigen::Matrix<double, 6, 6> get_adjoint(const Eigen::Matrix4d& frame);
    inline std::array<double, 36> get_adjoint(const std::array<double, 16>& frame);

    inline Eigen::Matrix<double, 6, 1> twist_transform(const Eigen::Matrix4d& frameRefObj,
                                                       const Eigen::Matrix<double, 6, 1>& twistObj);
    inline std::array<double, 6> twist_transform(const std::array<double, 16>& frameRefObj,
                                                 const std::array<double, 6>& twistObj);

    inline Eigen::Matrix<double, 6, 1> wrench_transform(const Eigen::Matrix4d& frameRefObj,
                                                        const Eigen::Matrix<double, 6, 1>& wrenchObj);
    inline std::array<double, 6> wrench_transform(const std::array<double, 16>& frameRefObj,
                                                  const std::array<double, 6>& wrenchObj);

    inline Eigen::Matrix<double, 6, 1> twist_integral(const double& step, const Eigen::Matrix<double, 6, 1>& acc,
                                                      const Eigen::Matrix<double, 6, 1>& lastTwist,
                                                      double eps = epsilon);
    inline std::array<double, 6> twist_integral(const double& step, const std::array<double, 6>& acc,
                                                const std::array<double, 6>& lastTwist, double eps = epsilon);

    inline Eigen::Matrix4d frame_integral(const double& step, const Eigen::Matrix<double, 6, 1>& twistObj,
                                          const Eigen::Matrix4d& lastFrameRefObj, double eps = epsilon);
    inline std::array<double, 16> frame_integral(const double& step, const std::array<double, 6>& twistObj,
                                                 const std::array<double, 16>& lastFrameRefObj, double eps = epsilon);

    inline Eigen::VectorXd q_integral(const double& step, const Eigen::VectorXd& qdot, const Eigen::VectorXd& lastQ,
                                      double eps = epsilon);
    inline std::vector<double> q_integral(const double& step, const std::vector<double>& qdot,
                                          const std::vector<double>& lastQ, double eps = epsilon);

    inline Eigen::Matrix3d get_rot(const Eigen::Matrix4d& frame) { return frame.block<3, 3>(0, 0); };

    inline std::array<double, 9> get_rot(const std::array<double, 16>& frame) {
        return std::array<double, 9>{frame[0], frame[1], frame[2], frame[4], frame[5],
                                     frame[6], frame[8], frame[9], frame[10]};
    };

    inline Eigen::Vector3d get_p(const Eigen::Matrix4d& frame) { return frame.block<3, 1>(0, 3); };

    inline std::array<double, 3> get_p(const std::array<double, 16>& frame) {
        return std::array<double, 3>{frame[12], frame[13], frame[14]};
    };

    inline Eigen::Matrix4d get_frame(const Eigen::Vector3d& p, const Eigen::Matrix3d& rot) {
        Eigen::Matrix4d frame = Eigen::Matrix4d::Identity();
        frame.block<3, 3>(0, 0) = rot;
        frame.block<3, 1>(0, 3) = p;
        return frame;
    };

    inline std::array<double, 16> get_frame(const std::array<double, 3>& p, const std::array<double, 9>& rot) {
        return std::array<double, 16>{rot[0], rot[1], rot[2], 0, rot[3], rot[4], rot[5], 0,
                                      rot[6], rot[7], rot[8], 0, p[0],   p[1],   p[2],   1};
    };

    inline Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& p) {
        Eigen::Matrix3d m;
        m << 0, -p(2), p(1), p(2), 0, -p(0), -p(1), p(0), 0;
        return m;
    };

    inline std::array<double, 9> skew_symmetric(const std::array<double, 3>& vec) {
        std::array<double, 9> m;
        Eigen::Map<Eigen::MatrixXd> _m(m.data(), 3, 3);
        _m << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0;
        return m;
    };

    inline Eigen::Matrix3d rot_inverse(const Eigen::Matrix3d& rot) { return rot.transpose(); };

    inline std::array<double, 9> rot_inverse(const std::array<double, 9>& rot) {
        return std::array<double, 9>{rot[0], rot[3], rot[6], rot[1], rot[4], rot[7], rot[2], rot[5], rot[8]};
    };

    inline Eigen::Matrix4d frame_inverse(const Eigen::Matrix4d& frame) {
        Eigen::Matrix4d inv = frame;
        inv.block<3, 3>(0, 0).transposeInPlace();
        inv.block<3, 1>(0, 3) = -inv.block<3, 3>(0, 0) * frame.block<3, 1>(0, 3);
        return inv;
    };

    inline std::array<double, 16> frame_inverse(const std::array<double, 16>& frame) {
        std::array<double, 16> inv = frame;
        Eigen::Map<Eigen::Matrix4d> _inv(inv.data());
        _inv = frame_inverse(_inv);
        return inv;
    };

    inline Eigen::Vector3d rotate(const Eigen::Matrix3d& rot, const Eigen::Vector3d& vec) { return rot * vec; };

    inline std::array<double, 3> rotate(const std::array<double, 9>& rot, const std::array<double, 3>& vec) {
        std::array<double, 9> r = rot;
        Eigen::Map<Eigen::MatrixXd> _r(r.data(), 3, 3);
        std::array<double, 3> v = vec;
        Eigen::Map<Eigen::VectorXd> _v(v.data(), 3);
        _v = _r * _v;
        return v;
    };

    inline Eigen::Matrix<double, 6, 1> rotate(const Eigen::Matrix3d& rot, const Eigen::Matrix<double, 6, 1>& vec) {
        Eigen::Matrix<double, 6, 1> v;
        v.segment(0, 3) = rot * vec.segment(0, 3);
        v.segment(3, 3) = rot * vec.segment(3, 3);
        return v;
    };

    inline std::array<double, 6> rotate(const std::array<double, 9>& rot, const std::array<double, 6>& vec) {
        std::array<double, 9> r = rot;
        Eigen::Map<Eigen::MatrixXd> _r(r.data(), 3, 3);
        std::array<double, 6> v = vec;
        Eigen::Map<Eigen::VectorXd> _v(v.data(), 6);
        _v.segment(0, 3) = _r * _v.segment(0, 3);
        _v.segment(3, 3) = _r * _v.segment(3, 3);
        return v;
    };

    inline Eigen::Matrix4d frame_subtract(const Eigen::Matrix4d& frameRef, const Eigen::Matrix4d& frameObj) {
        return frame_inverse(frameRef) * frameObj;
    };

    inline std::array<double, 16> frame_subtract(const std::array<double, 16>& frameRef,
                                                 const std::array<double, 16>& frameObj) {
        std::array<double, 16> _frameRef = frameRef;
        std::array<double, 16> _frameObj = frameObj;

        std::array<double, 16> frameRelObj = frameObj;
        Eigen::Map<Eigen::Matrix4d> _frameRelObj(frameRelObj.data());
        _frameRelObj = frame_subtract(Eigen::Map<Eigen::Matrix4d>(_frameRef.data()),
                                      Eigen::Map<Eigen::Matrix4d>(_frameObj.data()));

        return frameRelObj;
    };

    inline std::pair<double, Eigen::Vector3d> get_angle_axis(const Eigen::Matrix3d& rot, double eps) {
        double angle{0};
        Eigen::Vector3d axis = Eigen::Vector3d::Zero();
        if (std::abs(rot.determinant() - 1) > eps)
            return std::make_pair(angle, axis);

        Eigen::AngleAxisd pairAngleAxis;
        pairAngleAxis.fromRotationMatrix(rot);
        angle = pairAngleAxis.angle();
        axis = pairAngleAxis.axis();
        return std::make_pair(angle, axis);
    }

    inline std::pair<double, std::array<double, 3>> get_angle_axis(const std::array<double, 9>& rot, double eps) {
        std::array<double, 9> _rot = rot;
        Eigen::Map<Eigen::Matrix3d> __rot(_rot.data());

        double angle{0};
        std::array<double, 3> axis{0, 0, 0};
        if (std::abs(__rot.determinant() - 1) > eps)
            return std::make_pair(angle, axis);

        Eigen::AngleAxisd _pairAngleAxis;
        _pairAngleAxis.fromRotationMatrix(__rot);
        angle = _pairAngleAxis.angle();
        axis = {_pairAngleAxis.axis()(0), _pairAngleAxis.axis()(1), _pairAngleAxis.axis()(2)};
        return std::make_pair(angle, axis);
    }

    inline Eigen::Matrix3d get_rot(const double& angle, const Eigen::Vector3d& axis, double eps) {
        if (std::abs(angle) <= eps)
            return Eigen::Matrix3d::Identity();

        double norm = axis.norm();
        if (norm <= eps)
            return Eigen::Matrix3d::Identity();

        Eigen::AngleAxisd rot(angle, axis / norm);
        return rot.toRotationMatrix();
    };

    inline std::array<double, 9> get_rot(const double& angle, const std::array<double, 3>& axis, double eps) {
        std::array<double, 9> rot{1, 0, 0, 0, 1, 0, 0, 0, 1};

        if (std::abs(angle) <= eps) {
            return rot;
        }

        Eigen::Vector3d _axis(axis[0], axis[1], axis[2]);
        double norm = _axis.norm();
        if (norm <= eps) {
            return rot;
        }

        _axis = _axis / norm;
        Eigen::AngleAxisd _rot(angle, _axis);

        Eigen::Map<Eigen::MatrixXd> matRot(rot.data(), 3, 3);
        matRot = _rot.toRotationMatrix();
        return rot;
    };

    inline Eigen::Matrix<double, 6, 1> frame2pose(const Eigen::Matrix4d& frame) {
        Eigen::Matrix<double, 6, 1> pose;
        pose.segment(0, 3) = frame.block<3, 1>(0, 3);

        auto pairAngleAxis = get_angle_axis(frame.block<3, 3>(0, 0));
        pose.segment(3, 3) = pairAngleAxis.first * pairAngleAxis.second;

        return pose;
    };

    inline std::array<double, 6> frame2pose(const std::array<double, 16>& frame) {
        std::array<double, 6> pose;
        std::array<double, 16> _frame = frame;
        Eigen::Map<Eigen::Matrix<double, 6, 1>> _pose(pose.data());
        _pose = frame2pose(Eigen::Map<Eigen::Matrix4d>(_frame.data()));
        return pose;
    };

    inline Eigen::Matrix4d pose2frame(const Eigen::Matrix<double, 6, 1>& pose) {
        Eigen::Matrix4d frame;
        frame = get_frame(pose.segment(0, 3), get_rot(pose.segment(3, 3).norm(), pose.segment(3, 3)));
        return frame;
    };

    inline std::array<double, 16> pose2frame(const std::array<double, 6>& pose) {
        std::array<double, 3> p{pose[0], pose[1], pose[2]};
        std::array<double, 3> axis{pose[3], pose[4], pose[5]};
        double angle = pow(pow(axis[0], 2) + pow(axis[1], 2) + pow(axis[2], 2), 0.5);

        std::array<double, 16> frame = get_frame(p, get_rot(angle, axis));

        return frame;
    };

    inline Eigen::Matrix<double, 6, 6> get_adjoint(const Eigen::Matrix4d& frame) {
        Eigen::Matrix<double, 6, 6> adjoint = Eigen::Matrix<double, 6, 6>::Zero();
        adjoint.block<3, 3>(0, 0) = frame.block<3, 3>(0, 0);
        adjoint.block<3, 3>(0, 3) = skew_symmetric(frame.block<3, 1>(0, 3)) * frame.block<3, 3>(0, 0);
        adjoint.block<3, 3>(3, 3) = frame.block<3, 3>(0, 0);
        return adjoint;
    };

    inline std::array<double, 36> get_adjoint(const std::array<double, 16>& frame) {
        std::array<double, 16> _frame = frame;
        Eigen::Map<Eigen::Matrix4d> __frame(_frame.data());

        std::array<double, 36> adjoint;
        Eigen::Map<Eigen::Matrix<double, 6, 6>> _adjoint(adjoint.data());
        _adjoint = get_adjoint(__frame);

        return adjoint;
    };

    inline Eigen::Matrix<double, 6, 1> twist_transform(const Eigen::Matrix4d& frameRefObj,
                                                       const Eigen::Matrix<double, 6, 1>& twistObj) {
        auto adjoint = get_adjoint(frameRefObj);

        Eigen::Matrix<double, 6, 1> twistRef;
        twistRef = adjoint * twistObj;

        return twistRef;
    };

    inline std::array<double, 6> twist_transform(const std::array<double, 16>& frameRefObj,
                                                 const std::array<double, 6>& twistObj) {
        std::array<double, 16> _frameRefObj = frameRefObj;
        Eigen::Map<Eigen::Matrix4d> __frameRefObj(_frameRefObj.data());

        std::array<double, 6> _twistObj = twistObj;
        Eigen::Map<Eigen::Matrix<double, 6, 1>> __twistObj(_twistObj.data());

        std::array<double, 6> twistRef;
        Eigen::Map<Eigen::Matrix<double, 6, 1>> _twistRef(twistRef.data());
        _twistRef = twist_transform(__frameRefObj, __twistObj);

        return twistRef;
    };

    inline Eigen::Matrix<double, 6, 1> wrench_transform(const Eigen::Matrix4d& frameRefObj,
                                                        const Eigen::Matrix<double, 6, 1>& wrenchObj) {
        auto adjoint = get_adjoint(frame_inverse(frameRefObj));

        Eigen::Matrix<double, 6, 1> wrenchRef;
        wrenchRef = adjoint.transpose() * wrenchObj;

        return wrenchRef;
    };
    inline std::array<double, 6> wrench_transform(const std::array<double, 16>& frameRefObj,
                                                  const std::array<double, 6>& wrenchObj) {
        std::array<double, 16> _frameRefObj = frameRefObj;
        Eigen::Map<Eigen::Matrix4d> __frameRefObj(_frameRefObj.data());

        std::array<double, 6> _wrenchObj = wrenchObj;
        Eigen::Map<Eigen::Matrix<double, 6, 1>> __wrenchObj(_wrenchObj.data());

        std::array<double, 6> wrenchRef;
        Eigen::Map<Eigen::Matrix<double, 6, 1>> _wrenchRef(wrenchRef.data());
        _wrenchRef = wrench_transform(__frameRefObj, __wrenchObj);

        return wrenchRef;
    };

    inline Eigen::Matrix<double, 6, 1> twist_integral(const double& step, const Eigen::Matrix<double, 6, 1>& acc,
                                                      const Eigen::Matrix<double, 6, 1>& lastTwist, double eps) {
        Eigen::Matrix<double, 6, 1> twist = lastTwist;

        if (acc.segment(0, 3).norm() * step >= eps)
            twist.segment(0, 3) += acc.segment(0, 3) * step;

        if (acc.segment(3, 3).norm() * step >= eps)
            twist.segment(3, 3) += acc.segment(0, 3) * step;

        return twist;
    }

    inline std::array<double, 6> twist_integral(const double& step, const std::array<double, 6>& acc,
                                                const std::array<double, 6>& lastTwist, double eps) {
        Eigen::VectorXd _acc(6);
        memcpy(_acc.data(), acc.data(), sizeof(std::array<double, 6>));  // _acc <<
        // acc[0],acc[1],acc[2],acc[3],acc[4],acc[5];

        std::array<double, 6> twist = lastTwist;
        Eigen::Map<Eigen::VectorXd> _twist(twist.data(), 6);

        _twist = twist_integral(step, _acc, _twist, eps);

        return twist;
    }

    inline Eigen::Matrix4d frame_integral(const double& step, const Eigen::Matrix<double, 6, 1>& twistObj,
                                          const Eigen::Matrix4d& lastFrameRefObj, double eps) {
        Eigen::Matrix4d frameDelta = Eigen::Matrix4d::Identity();
        frameDelta.block<3, 1>(0, 3) = twistObj.segment(0, 3) * step;

        double n = twistObj.segment(3, 3).norm() * step;
        if (n > eps) {
            frameDelta.block<3, 3>(0, 0) = get_rot(n, twistObj.segment(3, 3), eps);
        }
        return lastFrameRefObj * frameDelta;
    };

    inline std::array<double, 16> frame_integral(const double& step, const std::array<double, 6>& twistObj,
                                                 const std::array<double, 16>& lastFrameRefObj, double eps) {
        Eigen::Matrix<double, 6, 1> _twist;
        memcpy(_twist.data(), twistObj.data(), sizeof(std::array<double, 6>));
        Eigen::Matrix4d _lastFrame;
        memcpy(_lastFrame.data(), lastFrameRefObj.data(), sizeof(std::array<double, 16>));

        std::array<double, 16> frame;
        Eigen::Map<Eigen::Matrix4d> _frame(frame.data());
        _frame = frame_integral(step, _twist, _lastFrame, eps);

        return frame;
    };

    inline Eigen::VectorXd q_integral(const double& step, const Eigen::VectorXd& qdot, const Eigen::VectorXd& lastQ,
                                      double eps) {
        Eigen::VectorXd q(lastQ.size());
        q = lastQ;
        for (int i = 0; i != q.size(); ++i) {
            if (std::abs(qdot(i)) * step >= eps) {
                q(i) += qdot(i) * step;
            }
        }

        return q;
    }

    inline std::vector<double> q_integral(const double& step, const std::vector<double>& qdot,
                                          const std::vector<double>& lastQ, double eps) {
        std::vector<double> q(lastQ);
        for (int i = 0; i != q.size(); ++i) {
            if (std::abs(qdot[i]) * step >= eps) {
                q[i] += qdot[i] * step;
            }
        }

        return q;
    }
}  // namespace vp::math
#endif  // RIGID_BODY_DYNAMICS_H
