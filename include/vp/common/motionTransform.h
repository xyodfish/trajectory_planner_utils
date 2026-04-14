#ifndef _MOTION_TRANSFORM_H_
#define _MOTION_TRANSFORM_H_

#include <algorithm>
#include "dbg.h"
#include "rigidBodyMotion.h"

namespace vp::math {
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    // AR Tcp Pose: [x, y, z, rx, ry, rz],
    // x, y, z: position of tcp, /m
    // rx, ry, rz: Axis-Angle. For angle theta/rad and axis [ax, ay, az], [rx, ry, rz] = [theta*ax, theta*ay, theta*az]

    // Axis-Angle vector [rx, ry, rz],
    // rx, ry, rz: Axis-Angle. For angle theta/rad and axis [ax, ay, az], [rx, ry, rz] = [theta*ax, theta*ay, theta*az]

    // Angular velocity [wx, wy, wz] / rad/s

#pragma region transformation between pose and Homogeneous matrix
    /**
     * @brief AR Tcp Pose to Homogeneous matrix
     * @param pose AR Tcp Pose
     * @return Eigen::Isometry3d
     */
    inline Eigen::Isometry3d pose2Homogeneous(const Eigen::Matrix<double, 6, 1>& pose);
    /**
     * @brief pose to Homogeneous matrix
     * @param pose [x, y, z, qx, qy, qz, qw].  [qx, qy, qz, qw] is quaternion
     * @return Eigen::Isometry3d
     */
    inline Eigen::Isometry3d pose2Homogeneous(const Eigen::Matrix<double, 7, 1>& pose);
    /**
     * @brief AR Tcp Pose to Homogeneous matrix
     * @param pose AR Tcp Pose (std::vector<double>)
     * @return Eigen::Isometry3d
     */
    inline Eigen::Isometry3d pose2Homogeneous(const std::vector<double>& pose);
    /**
     * @brief Homogeneous matrix to pose
     * @param transformation Homogeneous matrix
     * @return Eigen::Matrix<double, 7, 1> [x, y, z, qx, qy, qz, qw].  [qx, qy, qz, qw] is quaternion
     */
    inline Eigen::Matrix<double, 7, 1> homogeneous2Pose(const Eigen::Isometry3d& transformation);
    /**
     * @brief Homogeneous matrix to AR Tcp Pose (std::vector<double>)
     * @param transformation Homogeneous matrix
     * @return std::vector<double>
     */
    inline std::vector<double> homogeneous2STDPose(const Eigen::Isometry3d& transformation);
    /**
     * @brief Homogeneous matrix to AR Tcp Pose (Vector6d)
     *
     * @param transformation
     * @return Vector6d
     */
    inline Vector6d homogeneous2EigenPose(const Eigen::Isometry3d& transformation);
#pragma endregion

#pragma region transformation between Axis-Angle vector to Eigen::AngleAxisd
    /**
     * @brief Axis-Angle vector to Eigen::AngleAxisd
     * @param vector [rx, ry, rz]: Axis-Angle. For angle theta/rad and axis [ax, ay, az], [rx, ry, rz] = [theta*ax, theta*ay, theta*az]
     * @return Eigen::AngleAxisd
     */
    inline Eigen::AngleAxisd vector3dToAngleAxis(Eigen::Vector3d vector);
    /**
     * @brief Eigen::AngleAxisd to Axis-Angle vector
     * @param rotation_vector Eigen::AngleAxisd
     * @return Eigen::Vector3d  [rx, ry, rz]: Axis-Angle. For angle theta/rad and axis [ax, ay, az], [rx, ry, rz] = [theta*ax, theta*ay, theta*az]
     */
    inline Eigen::Vector3d angleAxisToVector3d(Eigen::AngleAxisd rotation_vector);
#pragma endregion

#pragma region operation about AR Tcp Pose and quaternion
    /**
     * @brief Get the Quaternion From AR Tcp Pose object
     * @param pose AR Tcp Pose
     * @return Eigen::Quaterniond
     */
    inline Eigen::Quaterniond getQuaternionFromTcpPose(const std::vector<double>& pose);
    /**
     * @brief Set the AR Tcp Pose (rotation part) From Quaternion object
     * @param pose  AR Tcp Pose
     * @param q quaternion
     */
    inline void setTcpPoseFromQuaternion(std::vector<double>& pose, const Eigen::Quaterniond& q);

    inline void setVector6dPoseFromQuaternion(Vector6d& pose, const Eigen::Quaterniond& q);
#pragma endregion

#pragma region transformation about euler angles, axis-angle vector, Eigen::AngleAxisd and rotation matrix
    /**
     * @brief eulerAngle To AngelAxis, RotZ(theta_z) * RotY(theta_y) * RotX(theta_x)
     * @param eulerAngle [theta_x, theta_y, theta_z]
     * @return Eigen::AngleAxisd
     */
    inline Eigen::AngleAxisd eulerAngleToAngelAxis(const Eigen::Vector3d& eulerAngle);
    inline Eigen::Matrix<double, 6, 1> vectorToEigen(std::vector<double> arg);

    inline std::vector<double> vector6dToStdPose(Vector6d pose);
    /**
     * @brief AngelAxis To euler Angle , RotZ(theta_z) * RotY(theta_y) * RotX(theta_x)
     * @param rotation_vector AngelAxis
     * @return Eigen::Vector3d
     */
    inline Eigen::Vector3d angleAxisToEulerAngles(const Eigen::AngleAxisd& rotation_vector);
    /**
     * @brief euler angle to rotation matrix,  RotX(theta_x) * RotY(theta_y) * RotZ(theta_z)
     * @param eulerAngles [theta_z, theta_y, theta_x]
     * @return Eigen::Matrix3d
     */
    inline Eigen::Matrix3d eulerAngleToRotationMatrix(Eigen::Vector3d eulerAngles);
#pragma endregion

#pragma region operations about AR Tcp Pose and plan pose
    /**
     * @brief plan pose to AR Tcp Pose
     * @param plan_pose [x, y, z, euler_x, euler_y, euler_z]
     * @return Eigen::Matrix<double, 6, 1>
     */
    inline Eigen::Matrix<double, 6, 1> plan_pose_to_tcp_pose(const Eigen::Matrix<double, 6, 1>& plan_pose);
    /**
     * @brief AR Tcp Pose to plan pose
     * @param tcp_pose AR Tcp Pose
     * @return Eigen::Matrix<double, 6, 1> [x, y, z, euler_x, euler_y, euler_z]
     */
    inline Eigen::Matrix<double, 6, 1> tcp_pose_to_plan_pose(Eigen::Matrix<double, 6, 1> tcp_pose);
    /**
     * @brief set AR Tcp Pose from plan pose
     * @param plan_pose  [x, y, z, euler_x, euler_y, euler_z]
     * @param tcp_pose  AR Tcp Pose
     */
    inline void tcpPoseFromPlanState(const Eigen::Matrix<double, 6, 1>& plan_pose, std::vector<double>& tcp_pose);
    /**
     * @brief set plan pose from AR Tcp Pose
     * @param tcp_pose  AR Tcp Pose
     * @param plan_pose  [x, y, z, euler_x, euler_y, euler_z]
     */
    inline void planPoseFromTcpPose(const std::vector<double>& tcp_pose, Eigen::Matrix<double, 6, 1>& plan_pose);
#pragma endregion

    /**
     * @brief AR Tcp Pose interpolation function
     * @param from AR Tcp Pose, start pose
     * @param to AR Tcp Pose, end pose
     * @param t ratio, [0,1]
     * @return std::vector<double> AR Tcp Pose
     */
    inline std::vector<double> tcpPoseInterpolation(const std::vector<double>& from, const std::vector<double>& to,
                                                    double t);

#pragma region calculate a vector from a Matrix4d

    inline Eigen::Matrix<double, 6, 1> matrix4dToVector6d(const Eigen::Matrix4d& mat);
    inline std::vector<double> matrix4dToPose(const Eigen::Matrix4d& mat);

#pragma endregion

#pragma region calculate anguler velocity
    /**
     * @brief calculate anguler velocity using rotiontion matrix
     * @param R1 rotiontion matrix 1
     * @param R2 rotiontion matrix 2
     * @param dt time interval
     * @return Eigen::Vector3d
     */
    inline Eigen::Vector3d anguler_velocity_from_rotation(const Eigen::Matrix3d& R1, const Eigen::Matrix3d& R2,
                                                          double dt);
    /**
     * @brief calculate anguler velocity using Axis-Angle vector
     * @param v1 Axis-Angle vector 1
     * @param v2 Axis-Angle vector 2
     * @param dt time interval
     * @return Eigen::Vector3d
     */
    inline Eigen::Vector3d anguler_velocity_from_angleAxis(Eigen::Vector3d v1, Eigen::Vector3d v2, double dt);
    /**
     * @brief calculate anguler velocity using Eigen::AngleAxisd
     * @param a1 Eigen::AngleAxisd 1
     * @param a2 Eigen::AngleAxisd 2
     * @param dt time interval
     * @return Eigen::Vector3d
     */
    inline Eigen::Vector3d anguler_velocity_from_angleAxis(const Eigen::AngleAxisd& a1, const Eigen::AngleAxisd& a2,
                                                           double dt);
    /**
     * @brief calculate anguler velocity using quaternion
     * @param q1 quaternion 1
     * @param q2 quaternion 2
     * @param dt time interval
     * @return Eigen::Vector3d
     */
    inline Eigen::Vector3d anguler_vel_from_quaternion(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2,
                                                       double dt);
#pragma endregion

#pragma region calculate translation from two translations

    /**
     * @brief get the trans data(std::vector<double>(6)) from pf(AR pose) to pt(AR pose)
     *
     * @param pf
     * @param pt
     * @return std::vector<double>
     */
    inline std::vector<double> transFromTwoPose(std::vector<double> pf, std::vector<double> pt);

    /**
     * @brief tran given pose (Vector6d) to the return pose(Vector6d) by the static frame trans Eigen::Isometry3d
     *
     * @param pose
     * @param T_frame
     * @return std::vector<double>
     */
    inline std::vector<double> transformStdPose(const Vector6d& pose, const Eigen::Isometry3d& T_frame,
                                                bool _is_static);

    /**
     * @brief
     *
     * @param pose
     * @param T_frame
     * @return Vector6d
     */
    inline Vector6d tranformToEigenPose(const Vector6d& pose, const Eigen::Isometry3d& T_frame, bool _is_static);
#pragma endregion

    std::vector<double> getPoseError(const std::vector<double>& pose_1, const std::vector<double>& pose_2);
    std::vector<double> calculateVelocityFromPose(const std::vector<double>& pose_1, const std::vector<double>& pose_2,
                                                  double dt);

    std::vector<double> poseAddDelta(const std::vector<double> pose, const std::vector<double> vel, double dt);

    double twoVectorDistance(const std::vector<double>& v1, const std::vector<double>& v2);

    inline Eigen::Isometry3d pose2Homogeneous(const Eigen::Matrix<double, 6, 1>& pose) {
        Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();
        frame.rotate(get_rot(pose.segment(3, 3).norm(), pose.segment(3, 3)));
        frame.pretranslate(Eigen::Vector3d(pose(0), pose(1), pose(2)));
        return frame;
    }

    inline Eigen::Isometry3d pose2Homogeneous(const Eigen::Matrix<double, 7, 1>& pose) {
        Eigen::Quaterniond q(pose(6), pose(3), pose(4), pose(5));
        Eigen::Isometry3d frame(q);
        frame.pretranslate(Eigen::Vector3d(pose(0), pose(1), pose(2)));
        return frame;
    }

    inline Eigen::Isometry3d pose2Homogeneous(const std::vector<double>& pose) {
        return pose2Homogeneous(vectorToEigen(pose));
    }

    inline Eigen::Matrix<double, 7, 1> homogeneous2Pose(const Eigen::Isometry3d& transformation) {
        Eigen::Matrix<double, 7, 1> pose;
        auto rot = transformation.rotation();  //
        // auto                          rot_linear = transformation.linear();
        auto pretrans = transformation.translation();  // 转平移向量
        auto quaternion = Eigen::Quaterniond(rot);     // 转为四元数

        pose.head(3) = pretrans;
        pose.segment(3, 4) = quaternion.coeffs();

        return pose;
    }

    inline std::vector<double> homogeneous2STDPose(const Eigen::Isometry3d& transformation) {
        std::vector<double> pose(6);
        Eigen::Vector3d trans = transformation.translation();
        pose[0] = trans[0];
        pose[1] = trans[1];
        pose[2] = trans[2];

        Eigen::Matrix3d rot = transformation.rotation();
        Eigen::AngleAxisd angle_axis = Eigen::AngleAxisd(rot);
        Eigen::Vector3d aa_vec = angle_axis.axis() * angle_axis.angle();

        pose[3] = aa_vec[0];
        pose[4] = aa_vec[1];
        pose[5] = aa_vec[2];

        return pose;
    }

    inline Vector6d homogeneous2EigenPose(const Eigen::Isometry3d& transformation) {
        Vector6d pose;

        Eigen::Vector3d trans = transformation.translation();
        pose[0] = trans[0];
        pose[1] = trans[1];
        pose[2] = trans[2];

        Eigen::Matrix3d rot = transformation.rotation();
        Eigen::AngleAxisd angle_axis = Eigen::AngleAxisd(rot);
        Eigen::Vector3d aa_vec = angle_axis.axis() * angle_axis.angle();

        pose[3] = aa_vec[0];
        pose[4] = aa_vec[1];
        pose[5] = aa_vec[2];
        return pose;
    }

    /**
     * @brief 将一个vector3d (模长为角度 其他的分量为旋转方向 )转为Eigen 的 轴角 Eigen::AngleAxisd
     *
     * @param vector_
     * @return Eigen::AngleAxisd
     */
    inline Eigen::AngleAxisd vector3dToAngleAxis(Eigen::Vector3d vector) {
        Eigen::AngleAxisd rotation_vector(vector.norm(), vector.normalized());
        return rotation_vector;
    }

    inline Eigen::Vector3d angleAxisToVector3d(Eigen::AngleAxisd rotation_vector) {
        Eigen::Vector3d vector;
        vector = rotation_vector.axis();
        vector *= rotation_vector.angle();

        return vector;
    }

    inline Eigen::Quaterniond getQuaternionFromTcpPose(const std::vector<double>& pose) {
        Eigen::Vector3d angle;
        angle(0) = pose.at(3);
        angle(1) = pose.at(4);
        angle(2) = pose.at(5);

        auto rv = vector3dToAngleAxis(angle);

        Eigen::Quaterniond quaternion(rv);

        return quaternion;
    }

    inline Eigen::Quaterniond getQuaternionFromTcpPose(const Eigen::Matrix<double, 6, 1>& pose) {
        Eigen::Vector3d angle;
        angle(0) = pose(3);
        angle(1) = pose(4);
        angle(2) = pose(5);

        auto rv = vector3dToAngleAxis(angle);
        Eigen::Quaterniond quaternion(rv);

        return quaternion;
    }

    inline void setTcpPoseFromQuaternion(std::vector<double>& pose, const Eigen::Quaterniond& q) {
        Eigen::AngleAxisd rotation_vector(q);
        Eigen::Vector3d vector = angleAxisToVector3d(rotation_vector);

        pose.at(3) = vector(0);
        pose.at(4) = vector(1);
        pose.at(5) = vector(2);
    }

    inline void setVector6dPoseFromQuaternion(Vector6d& pose, const Eigen::Quaterniond& q) {
        Eigen::AngleAxisd rotation_vector(q);
        Eigen::Vector3d vector = angleAxisToVector3d(rotation_vector);

        pose[3] = vector(0);
        pose[4] = vector(1);
        pose[5] = vector(2);
    }

    /**
     * @brief 将欧拉角转为轴角度 旋转向量按照 Z-Y-X 的顺序旋转 即RPY的顺序
     *
     * @param eulerAngle
     * @return Eigen::AngleAxisd 轴角
     */
    inline Eigen::AngleAxisd eulerAngleToAngelAxis(const Eigen::Vector3d& eulerAngle) {
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitZ()));

        Eigen::AngleAxisd rotation_vector;
        rotation_vector = yawAngle * pitchAngle * rollAngle;

        return rotation_vector;
    }

    inline Eigen::Matrix<double, 6, 1> vectorToEigen(std::vector<double> arg) {
        return (Eigen::Map<Eigen::Matrix<double, 6, 1>>(arg.data()));
    }

    inline std::vector<double> vector6dToStdPose(Vector6d pose) {
        return std::vector<double>(&pose[0], pose.data() + pose.cols() * pose.rows());
    }

    /**
     * @brief 旋转向量转为欧拉角度 按照 Z-Y-X 的顺序旋转 即RPY的顺序
     *
     * @param rotation_vector
     * @return Eigen::Vector3d euelrAngle
     */
    inline Eigen::Vector3d angleAxisToEulerAngles(const Eigen::AngleAxisd& rotation_vector) {
        Eigen::Vector3d eulerAngle = rotation_vector.matrix().eulerAngles(2, 1, 0);
        return eulerAngle;
    }

    /**
     * @brief 欧拉角转为旋转矩阵
     *
     * @param eulerAngles
     * @return Eigen::Matrix3d
     */
    inline Eigen::Matrix3d eulerAngleToRotationMatrix(Eigen::Vector3d eulerAngles) {
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix = Eigen::AngleAxisd(eulerAngles[2], Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(eulerAngles[1], Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(eulerAngles[0], Eigen::Vector3d::UnitZ());
        return rotation_matrix;
    }

    inline Eigen::Matrix<double, 6, 1> plan_pose_to_tcp_pose(const Eigen::Matrix<double, 6, 1>& plan_pose) {
        Eigen::Matrix<double, 6, 1> tcp_pose = plan_pose.segment(0, 6);
        Eigen::AngleAxisd rotation_vector = eulerAngleToAngelAxis(tcp_pose.segment(3, 3));
        tcp_pose.segment(3, 3) = rotation_vector.angle() * rotation_vector.axis();
        return tcp_pose;
    }

    inline Eigen::Matrix<double, 6, 1> tcp_pose_to_plan_pose(Eigen::Matrix<double, 6, 1> tcp_pose) {
        auto ret = tcp_pose;
        auto rotation_vector = vector3dToAngleAxis(tcp_pose.segment(3, 3));
        ret.segment(3, 3) = angleAxisToEulerAngles(rotation_vector);
        return ret;
    }

    inline void tcpPoseFromPlanState(const Eigen::Matrix<double, 6, 1>& plan_pose, std::vector<double>& tcp_pose) {
        auto tmp_pose = plan_pose_to_tcp_pose(plan_pose);
        tcp_pose.assign(tmp_pose.data(), tmp_pose.data() + tmp_pose.size());
    }

    inline void planPoseFromTcpPose(const std::vector<double>& tcp_pose, Eigen::Matrix<double, 6, 1>& plan_pose) {
        auto tcp_state = vectorToEigen(tcp_pose);
        plan_pose = tcp_pose_to_plan_pose(tcp_state);
    }

    inline Eigen::Matrix<double, 6, 1> matrix4dToVector6d(const Eigen::Matrix4d& mat) {
        Eigen::Matrix<double, 6, 1> ret_pose;
        ret_pose.head(3) = mat.topRightCorner<3, 1>();

        Eigen::AngleAxisd aa;
        aa.fromRotationMatrix(mat.block<3, 3>(0, 0));
        ret_pose.tail(3) = aa.angle() * aa.axis();
        return ret_pose;
    }

    inline std::vector<double> matrix4dToPose(const Eigen::Matrix4d& mat) {
        auto ret_pose = matrix4dToVector6d(mat);
        std::vector<double> ret_vec(&ret_pose[0], ret_pose.data() + ret_pose.cols() * ret_pose.rows());
        return ret_vec;
    }

    /****************************************************根据两个姿态计算角速度***************************************************/
    inline Eigen::Vector3d anguler_vel_from_rotation(const Eigen::Matrix3d& R1, const Eigen::Matrix3d& R2, double dt) {
        Eigen::AngleAxisd ax(R2 * R1.inverse());
        return (1 / dt) * (ax.angle() * ax.axis());
    }

    inline Eigen::Vector3d anguler_velocity_from_angleAxis(Eigen::Vector3d v1, Eigen::Vector3d v2, double dt) {
        Eigen::AngleAxisd a1(v1.norm(), v1.normalized());
        Eigen::AngleAxisd a2(v2.norm(), v2.normalized());

        return anguler_velocity_from_angleAxis(a1, a2, dt);
    }

    inline Eigen::Vector3d anguler_velocity_from_angleAxis(const Eigen::AngleAxisd& a1, const Eigen::AngleAxisd& a2,
                                                           double dt) {
        Eigen::Matrix3d R1(a1.toRotationMatrix());
        Eigen::Matrix3d R2(a2.toRotationMatrix());

        return anguler_vel_from_rotation(R1, R2, dt);
    }

    inline Eigen::Vector3d anguler_vel_from_quaternion(const Eigen::Quaterniond& qf, const Eigen::Quaterniond& qt,
                                                       double dt) {
        Eigen::Matrix3d R1(qf.toRotationMatrix());
        Eigen::Matrix3d R2(qt.toRotationMatrix());

        return anguler_vel_from_rotation(R1, R2, dt);
    }

    /**
     * @brief 计算旋转偏差
     *
     * @param v1
     * @param v2
     * @return Eigen::Vector3d
     */
    inline Eigen::Vector3d delat_aa_vec_from_two_aa_vec(Eigen::Vector3d v1, Eigen::Vector3d v2) {
        ///
        Eigen::AngleAxisd a1(v1.norm(), v1.normalized());
        Eigen::AngleAxisd a2(v2.norm(), v2.normalized());

        return anguler_velocity_from_angleAxis(a1, a2, 1.0);
    }

    inline Vector6d get_delta_pose(Vector6d pose1, Vector6d pose2) {
        Vector6d delta_pos;
        delta_pos.head(3) = pose1.head(3) - pose2.head(3);

        // 计算姿态偏差
        delta_pos.tail(3) = vp::math::delat_aa_vec_from_two_aa_vec(pose1.tail(3), pose2.tail(3));
        // delta_pos.tail(3) = pose1.tail(3) - pose2.tail(3);

        return delta_pos;
    }

    inline Vector6d get_delta_pose(const std::vector<double>& pose1, const std::vector<double>& pose2) {
        return get_delta_pose(vectorToEigen(pose1), vectorToEigen(pose2));
    }

    inline std::vector<double> tcpPoseInterpolation(const std::vector<double>& from, const std::vector<double>& to,
                                                    double t) {
        t = std::min(std::max(0.0, t), 1.0);
        std::vector<double> result = from;
        Eigen::Quaterniond q_from = vp::math::getQuaternionFromTcpPose(from);
        Eigen::Quaterniond q_to = vp::math::getQuaternionFromTcpPose(to);

        for (int i = 0; i < 3; i++)
            result[i] += (to[i] - from[i]) * t;

        Eigen::Quaterniond q_temp = q_from.slerp(t, q_to);

        vp::math::setTcpPoseFromQuaternion(result, q_temp);
        return result;
    }

    inline Vector6d vector6dPoseInterpolation(const Vector6d& from, const Vector6d& to, double t) {
        t = std::min(std::max(0.0, t), 1.0);
        Vector6d result = from;
        Eigen::Quaterniond q_from = vp::math::getQuaternionFromTcpPose(from);
        Eigen::Quaterniond q_to = vp::math::getQuaternionFromTcpPose(to);

        for (int i = 0; i < 3; i++)
            result[i] += (to[i] - from[i]) * t;

        Eigen::Quaterniond q_temp = q_from.slerp(t, q_to);

        vp::math::setVector6dPoseFromQuaternion(result, q_temp);
        return result;
    }

    inline double getTwoVectorAngles(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
        return std::acos(v1.dot(v2) / (v1.norm() * v2.norm()));
    }

    inline std::vector<double> getPoseError(const std::vector<double>& pose_1, const std::vector<double>& pose_2) {
        return calculateVelocityFromPose(pose_1, pose_2, 1);
    }

    inline std::vector<double> calculateVelocityFromPose(const std::vector<double>& pose_1,
                                                         const std::vector<double>& pose_2, double dt) {
        std::vector<double> vel(6, 0);

        for (int i = 0; i < 3; i++) {
            vel[i] = (pose_2[i] - pose_1[i]) / dt;

            double temp = pose_1[i] + vel[i] * dt;

            double a = temp + 1;
        }

        Eigen::Vector3d rv_1(pose_1[3], pose_1[4], pose_1[5]);
        Eigen::Vector3d rv_2(pose_2[3], pose_2[4], pose_2[5]);

        Eigen::Vector3d orien_vel = anguler_velocity_from_angleAxis(rv_1, rv_2, dt);

        for (int i = 0; i < 3; i++) {
            vel[i + 3] = orien_vel[i];
        }

        return vel;
    }

    inline std::vector<double> poseAddDelta(const std::vector<double> pose, const std::vector<double> vel, double dt) {
        std::vector<double> res = vel;

        for (int i = 0; i < 6; i++) {
            res[i] *= dt;
        }

        Eigen::Isometry3d T_pose = pose2Homogeneous(pose);
        Eigen::Isometry3d T_rot = pose2Homogeneous(res);

        T_pose = T_rot * T_pose;

        res = homogeneous2STDPose(T_pose);

        for (int i = 0; i < 3; i++) {
            res[i] = pose[i] + vel[i] * dt;
        }

        return res;
    }

    /**
     * @brief 根据两个给定坐标变换,(pf、pt) 描述frame_f->frame_t的坐标变换
     *
     * @param pf (frame_middle->frame_from的坐标变换)
     * @param pt (frame_middle->frame_to的坐标变换)
     * @return std::vector<double>
     */
    inline std::vector<double> transFromTwoPose(std::vector<double> pf, std::vector<double> pt) {
        Eigen::Isometry3d T_pf = pose2Homogeneous(pf).inverse();
        Eigen::Isometry3d T_pt = pose2Homogeneous(pt);

        Eigen::Isometry3d T_f2t = T_pt * T_pf;

        return homogeneous2STDPose(T_f2t);
    }

    inline std::vector<double> transformStdPose(const Vector6d& pose, const Eigen::Isometry3d& T_frame,
                                                bool _is_static = true) {

        Eigen::Isometry3d T_pose;

        if (_is_static) {
            T_pose = T_frame * pose2Homogeneous(pose);
        } else {
            T_pose = pose2Homogeneous(pose) * T_frame;
        }

        return homogeneous2STDPose(T_pose);
    }

    inline Vector6d tranformToEigenPose(const Vector6d& pose, const Eigen::Isometry3d& T_frame,
                                        bool _is_static = true) {
        auto tmp_pose = transformStdPose(pose, T_frame, _is_static);
        return vectorToEigen(tmp_pose);
    }

    inline double twoVectorDistance(const std::vector<double>& v1, const std::vector<double>& v2) {
        return (vp::math::vectorToEigen(v1) - vp::math::vectorToEigen(v2)).head(3).norm();
    }

    /// @brief rpy转变换矩阵
    /// @param _pose (x,y,z,r,p,y)
    /// @return 变换矩阵 Eigen::Isometry3d

    inline Eigen::Isometry3d rpyToMatrix(const std::vector<double>& _xyz, const std::vector<double>& _rpy) {
        Eigen::Isometry3d mat = Eigen::Isometry3d::Identity();

        if ((_xyz.size() != 3) || (_rpy.size() != 3)) {
            std::cerr << "The format of rpy is wrong!" << std::endl;
        } else {
            Eigen::Vector3d translation(_xyz.at(0), _xyz.at(1), _xyz.at(2));
            Eigen::Matrix3d rotation;
            rotation = Eigen::AngleAxisd(_rpy.at(2), Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(_rpy.at(1), Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(_rpy.at(0), Eigen::Vector3d::UnitX());
            mat.linear() = rotation.matrix();
            mat.translation() = translation;
        }

        return mat;
    };

    inline Eigen::Isometry3d rpyToMatrix(const std::vector<double>& _pose) {
        std::vector<double> xyz{_pose.at(0), _pose.at(1), _pose.at(2)};
        std::vector<double> rpy{_pose.at(3), _pose.at(4), _pose.at(5)};

        return rpyToMatrix(xyz, rpy);
    }

}  // namespace vp::math

#endif
