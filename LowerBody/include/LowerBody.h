#ifndef LOWER_BODY_H
#define LOWER_BODY_H

#include <Eigen/Dense>

/// 170C 腰腿联动解析正逆运动学。
///
/// 纯运动学库：无 CAN、无规划、无驱动。角度一律为弧度，长度为米。
/// TCP 位姿约定与 T3_leg 相同：[x, y, z, rx, ry, rz]（ZYX 欧拉）。
///
/// 关节 q[5] 顺序见 Joint：
///   [脚踝, 膝盖, 髋电机, 腰 roll, 腰 yaw]
/// 髋电机为腿髋与腰 pitch 的共用轴：q_hip = q_leg_hip − q_pitch。
class LowerBody {
public:
    /// 默认几何（单位 m）。部署时若实物尺寸不同，构造时传入 Params。
    struct Params {
        double L1 = 0.260;  ///< 小腿（脚踝 → 膝盖）
        double L2 = 0.400;  ///< 大腿（膝盖 → 髋）
        double d2 = 0.110;  ///< 腰 Q1 → Q2 偏置；零位沿基座 +Z
    };

    /// 5 维电机角下标。
    enum Joint {
        Ankle = 0,      ///< 脚踝，电机轴沿基座 −Y
        Knee = 1,       ///< 膝盖，电机轴沿基座 −Y
        Hip = 2,        ///< 共用髋电机（腿髋 − 腰 pitch），轴沿基座 −Y
        WaistRoll = 3,  ///< 腰侧倾，轴沿基座 +X
        WaistYaw = 4,   ///< 腰回转，轴沿基座 +Z
        Dof = 5
    };

    using Joints3 = Eigen::Matrix<double, 1, 3>;  ///< 腿 [脚踝, 膝盖, 腿髋] 或腰 [pitch, roll, yaw]
    using Joints5 = Eigen::Matrix<double, 1, 5>;  ///< 电机角，见 Joint
    using Pose6 = Eigen::Matrix<double, 1, 6>;    ///< [x, y, z, rx, ry, rz]，m / rad

    LowerBody();
    explicit LowerBody(Params params);

    const Params& params() const { return params_; }

    /// 电机角 → TCP 齐次变换。
    /// 位置 = 髋点 + 点头引起的 Q2 相对零位偏移 (cx, 0, cz)；
    /// 姿态 = Ry(pitch) Rx(roll) Rz(yaw)。
    Eigen::Matrix4d Forward_Kinematics(const Joints5& q) const;

    /// 电机角 → TCP 位姿 [x, y, z, rx, ry, rz]。
    Pose6 Forward_Kinematics_Pose(const Joints5& q) const;

    /// TCP 齐次变换 → 电机角。
    /// 腿不可达时脚踝/膝盖/髋为 NaN（腰仍可能有值）。用 Reachable() 判断。
    Joints5 Inverse_Kinematics(const Eigen::Matrix4d& T) const;

    /// TCP 位姿 → 电机角。
    Joints5 Inverse_Kinematics_Pose(const Pose6& pose) const;

    /// 腰姿态 → [pitch, roll, yaw]，并给出 Q2 相对零位的基座 X/Z 偏移。
    /// cx = d2·sin(pitch)，cz = d2·(cos(pitch)−1)。逆解腿时从目标位置减去该偏移。
    Joints3 Inverse_Waist(const Eigen::Matrix3d& R, double& comp_x, double& comp_z) const;

    /// 髋点位置（髋姿态固定为单位阵）→ [脚踝, 膝盖, 腿髋]。
    /// 选取膝盖弯向 −X 的那一支；不可达则全 NaN。
    Joints3 Inverse_Leg(const Eigen::Vector3d& p_hip) const;

    /// 脚踝、膝盖 → 髋点位置（髋姿态保持单位阵）。
    Eigen::Vector3d Forward_Hip(double q_ankle, double q_knee) const;

    /// 拆出腿几何角与腰欧拉角。
    /// q_leg = [脚踝, 膝盖, 腿髋]，其中 腿髋 = −脚踝 − 膝盖（使髋姿态为 I）；
    /// q_waist = [pitch, roll, yaw]，其中 pitch = 腿髋 − q[Hip]。
    void Split(const Joints5& q, Joints3& q_leg, Joints3& q_waist) const;

    /// 腿三轴均为有限值则为可达。
    static bool Reachable(const Joints5& q);

    /// 齐次变换 ↔ ZYX 位姿（与 T3_leg 相同：R = Rz(rz) Ry(ry) Rx(rx)）。
    static Pose6 Pose_From_Transform(const Eigen::Matrix4d& T);
    static Eigen::Matrix4d Transform_From_Pose(const Pose6& pose);

    /// 折到 (−π, π]。
    static double WrapPi(double a);

private:
    Params params_;

    static Eigen::Matrix3d Rx(double a);
    static Eigen::Matrix3d Ry(double a);
    static Eigen::Matrix3d Rz(double a);
};

#endif  // LOWER_BODY_H
