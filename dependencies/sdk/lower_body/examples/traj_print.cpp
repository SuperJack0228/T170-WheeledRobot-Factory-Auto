/// 轨迹抽帧打印（核对用，非规划器）。
///
/// 第一段：关节插值  q0 → IK(pose1)
/// 第二段：位姿插值  pose1 → pose2
///
/// 列：关节角 rad；髋 / Q2 位置 mm；髋 / Q2 欧拉角 rad（ZYX）。
#include "LowerBody.h"

#include <cmath>
#include <cstdio>

namespace {

constexpr int kFrames = 21;
constexpr double kMm = 1.0e3;

LowerBody::Joints5 LerpJoints(const LowerBody::Joints5& a, const LowerBody::Joints5& b, double t)
{
    LowerBody::Joints5 q;
    for (int i = 0; i < LowerBody::Dof; ++i) {
        q(i) = LowerBody::WrapPi(a(i) + t * LowerBody::WrapPi(b(i) - a(i)));
    }
    return q;
}

Eigen::Matrix4d LerpPose(const Eigen::Matrix4d& A, const Eigen::Matrix4d& B, double t)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 1>(0, 3) = (1.0 - t) * A.block<3, 1>(0, 3) + t * B.block<3, 1>(0, 3);
    const Eigen::Quaterniond qa(A.block<3, 3>(0, 0));
    const Eigen::Quaterniond qb(B.block<3, 3>(0, 0));
    T.block<3, 3>(0, 0) = qa.slerp(t, qb).toRotationMatrix();
    return T;
}

void PrintHeader()
{
    std::printf("%3s %8s %8s %8s %8s %8s"
                " %8s %8s %8s %8s %8s %8s"
                " %8s %8s %8s %8s %8s %8s\n",
                "i", "ank", "knee", "hip", "roll", "yaw", "hx", "hy", "hz", "hrx", "hry", "hrz",
                "q2x", "q2y", "q2z", "q2rx", "q2ry", "q2rz");
}

void PrintRow(const LowerBody& kin, int i, const LowerBody::Joints5& q)
{
    if (!LowerBody::Reachable(q)) {
        std::printf("%3d %8s %8s %8s %8s %8s"
                    " %8s %8s %8s %8s %8s %8s"
                    " %8s %8s %8s %8s %8s %8s\n",
                    i, "nan", "nan", "nan", "nan", "nan", "nan", "nan", "nan", "nan", "nan", "nan",
                    "nan", "nan", "nan", "nan", "nan", "nan");
        return;
    }

    LowerBody::Joints3 q_leg, q_waist;
    kin.Split(q, q_leg, q_waist);
    const Eigen::Vector3d p_hip = kin.Forward_Hip(q_leg(0), q_leg(1));
    const double pitch = q_waist(0);
    const double roll = q_waist(1);
    const double d2 = kin.params().d2;
    const Eigen::Vector3d p_q2 =
        p_hip + Eigen::Vector3d(d2 * std::sin(pitch), 0.0, d2 * std::cos(pitch));

    Eigen::Matrix4d T_hip = Eigen::Matrix4d::Identity();
    T_hip.block<3, 1>(0, 3) = p_hip;
    const LowerBody::Pose6 hip = LowerBody::Pose_From_Transform(T_hip);

    Eigen::Matrix3d R_q2 =
        (Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(q_waist(2), Eigen::Vector3d::UnitZ()))
            .toRotationMatrix();
    Eigen::Matrix4d T_q2 = Eigen::Matrix4d::Identity();
    T_q2.block<3, 3>(0, 0) = R_q2;
    T_q2.block<3, 1>(0, 3) = p_q2;
    const LowerBody::Pose6 q2 = LowerBody::Pose_From_Transform(T_q2);

    std::printf("%3d %8.4f %8.4f %8.4f %8.4f %8.4f"
                " %8.1f %8.1f %8.1f %8.4f %8.4f %8.4f"
                " %8.1f %8.1f %8.1f %8.4f %8.4f %8.4f\n",
                i, q(LowerBody::Ankle), q(LowerBody::Knee), q(LowerBody::Hip),
                q(LowerBody::WaistRoll), q(LowerBody::WaistYaw), hip(0) * kMm, hip(1) * kMm,
                hip(2) * kMm, hip(3), hip(4), hip(5), q2(0) * kMm, q2(1) * kMm, q2(2) * kMm,
                q2(3), q2(4), q2(5));
}

}  // namespace

int main()
{
    const LowerBody kin;
    PrintHeader();

    LowerBody::Joints5 q0;
    q0 << M_PI / 2.0, -M_PI / 2.0, M_PI / 2.0, 0.0, 0.0;
    LowerBody::Pose6 pose1;
    pose1 << 0.0, 0.0, 0.400, 0.0, 0.0, 0.0;
    const LowerBody::Joints5 q1 = kin.Inverse_Kinematics_Pose(pose1);
    for (int i = 0; i < kFrames; ++i) {
        PrintRow(kin, i, LerpJoints(q0, q1, i / double(kFrames - 1)));
    }

    std::printf("\n");
    PrintHeader();

    LowerBody::Pose6 pose2;
    pose2 << -0.100, 0.0, 0.500, 0.0, 0.7, 0.7;
    const Eigen::Matrix4d Ta = LowerBody::Transform_From_Pose(pose1);
    const Eigen::Matrix4d Tb = LowerBody::Transform_From_Pose(pose2);
    for (int i = 0; i < kFrames; ++i) {
        PrintRow(kin, i, kin.Inverse_Kinematics(LerpPose(Ta, Tb, i / double(kFrames - 1))));
    }
    return 0;
}
