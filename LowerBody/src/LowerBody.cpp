#include "LowerBody.h"

#include <cmath>
#include <limits>

LowerBody::LowerBody() : params_{} {}

LowerBody::LowerBody(Params params) : params_(params) {}

double LowerBody::WrapPi(double a)
{
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0.0) {
        a += 2.0 * M_PI;
    }
    return a - M_PI;
}

bool LowerBody::Reachable(const Joints5& q)
{
    return q.head<3>().allFinite();
}

Eigen::Matrix3d LowerBody::Rx(double a)
{
    const double c = std::cos(a), s = std::sin(a);
    Eigen::Matrix3d R;
    R << 1, 0, 0, 0, c, -s, 0, s, c;
    return R;
}

Eigen::Matrix3d LowerBody::Ry(double a)
{
    const double c = std::cos(a), s = std::sin(a);
    Eigen::Matrix3d R;
    R << c, 0, s, 0, 1, 0, -s, 0, c;
    return R;
}

Eigen::Matrix3d LowerBody::Rz(double a)
{
    const double c = std::cos(a), s = std::sin(a);
    Eigen::Matrix3d R;
    R << c, -s, 0, s, c, 0, 0, 0, 1;
    return R;
}

LowerBody::Pose6 LowerBody::Pose_From_Transform(const Eigen::Matrix4d& T)
{
    const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    const double sy = std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
    double rx = 0.0, ry = 0.0, rz = 0.0;
    if (sy >= 1e-6) {
        rx = std::atan2(R(2, 1), R(2, 2));
        ry = std::atan2(-R(2, 0), sy);
        rz = std::atan2(R(1, 0), R(0, 0));
    } else {
        rx = std::atan2(-R(1, 2), R(1, 1));
        ry = std::atan2(-R(2, 0), sy);
    }
    Pose6 pose;
    pose << T(0, 3), T(1, 3), T(2, 3), rx, ry, rz;
    return pose;
}

Eigen::Matrix4d LowerBody::Transform_From_Pose(const Pose6& pose)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rz(pose(5)) * Ry(pose(4)) * Rx(pose(3));
    T(0, 3) = pose(0);
    T(1, 3) = pose(1);
    T(2, 3) = pose(2);
    return T;
}

Eigen::Vector3d LowerBody::Forward_Hip(double q_ankle, double q_knee) const
{
    // 电机正转 = Ry(−q)：q=0 时连杆沿 +Z，q>0 时弯向 −X。
    const Eigen::Vector3d zL1(0.0, 0.0, params_.L1);
    const Eigen::Vector3d zL2(0.0, 0.0, params_.L2);
    return Ry(-q_ankle) * zL1 + Ry(-q_ankle) * Ry(-q_knee) * zL2;
}

void LowerBody::Split(const Joints5& q, Joints3& q_leg, Joints3& q_waist) const
{
    const double q_ankle = q(Ankle);
    const double q_knee = q(Knee);
    const double q_leg_hip = WrapPi(-q_ankle - q_knee);
    const double q_pitch = WrapPi(q_leg_hip - q(Hip));
    q_leg << q_ankle, q_knee, q_leg_hip;
    q_waist << q_pitch, q(WaistRoll), q(WaistYaw);
}

Eigen::Matrix4d LowerBody::Forward_Kinematics(const Joints5& q) const
{
    Joints3 q_leg, q_waist;
    Split(q, q_leg, q_waist);
    const double q_pitch = q_waist(0);
    const Eigen::Vector3d p_hip = Forward_Hip(q_leg(0), q_leg(1));
    const double cx = params_.d2 * std::sin(q_pitch);
    const double cz = params_.d2 * (std::cos(q_pitch) - 1.0);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Ry(q_pitch) * Rx(q_waist(1)) * Rz(q_waist(2));
    T.block<3, 1>(0, 3) = p_hip + Eigen::Vector3d(cx, 0.0, cz);
    return T;
}

LowerBody::Pose6 LowerBody::Forward_Kinematics_Pose(const Joints5& q) const
{
    return Pose_From_Transform(Forward_Kinematics(q));
}

LowerBody::Joints3 LowerBody::Inverse_Waist(const Eigen::Matrix3d& R, double& comp_x,
                                            double& comp_z) const
{
    // R = Ry(pitch) Rx(roll) Rz(yaw)
    const double c2 = std::sqrt(std::max(0.0, R(1, 0) * R(1, 0) + R(1, 1) * R(1, 1)));
    Joints3 q;
    q << WrapPi(std::atan2(R(0, 2), R(2, 2))),
         WrapPi(std::atan2(-R(1, 2), c2)),
         WrapPi(std::atan2(R(1, 0), R(1, 1)));
    comp_x = params_.d2 * std::sin(q(0));
    comp_z = params_.d2 * (std::cos(q(0)) - 1.0);
    return q;
}

LowerBody::Joints3 LowerBody::Inverse_Leg(const Eigen::Vector3d& p_hip) const
{
    const double L1 = params_.L1;
    const double L2 = params_.L2;
    const double x = p_hip.x();
    const double z = p_hip.z();
    const double c_k = (x * x + z * z - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);

    Joints3 q_fail = Joints3::Constant(std::numeric_limits<double>::quiet_NaN());
    if (c_k < -1.0 - 1e-9 || c_k > 1.0 + 1e-9) {
        return q_fail;
    }
    const double ck = std::max(-1.0, std::min(1.0, c_k));
    const double ak = std::acos(ck);

    Joints3 best_q = q_fail;
    double best_knee_x = std::numeric_limits<double>::infinity();
    const double qk_cands[2] = {ak, -ak};
    for (double q_k : qk_cands) {
        const double A = L1 + L2 * std::cos(q_k);
        const double B = L2 * std::sin(q_k);
        const double q_a = WrapPi(std::atan2(-A * x - B * z, -B * x + A * z));
        const double knee_x = -L1 * std::sin(q_a);
        if (knee_x < best_knee_x) {
            best_knee_x = knee_x;
            best_q << q_a, WrapPi(q_k), WrapPi(-q_a - q_k);
        }
    }
    return best_q;
}

LowerBody::Joints5 LowerBody::Inverse_Kinematics(const Eigen::Matrix4d& T) const
{
    double comp_x = 0.0;
    double comp_z = 0.0;
    const Joints3 q_w = Inverse_Waist(T.block<3, 3>(0, 0), comp_x, comp_z);

    Eigen::Vector3d p = T.block<3, 1>(0, 3);
    p.x() -= comp_x;
    p.z() -= comp_z;
    const Joints3 q_l = Inverse_Leg(p);

    Joints5 q;
    q << q_l(0), q_l(1), WrapPi(q_l(2) - q_w(0)), q_w(1), q_w(2);
    return q;
}

LowerBody::Joints5 LowerBody::Inverse_Kinematics_Pose(const Pose6& pose) const
{
    return Inverse_Kinematics(Transform_From_Pose(pose));
}
