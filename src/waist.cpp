#include "waist.h"
#include "Ti5_socketcan.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>

namespace
{
const char *kJointName[dof_waist] = {"脚踝", "膝盖", "髋", "侧倾", "回转"};
/** 几何 = direct × 电机。降 Z 往后坐：库踝正/膝负/髋正 → 电机 4 负 / 3 正 / 2 正。 */
constexpr double kMotorDirect[dof_waist] = {-1.0, -1.0, 1.0, 1.0, 1.0};

LowerBody::Joints5 apply_motor_direct(const LowerBody::Joints5 &q)
{
    LowerBody::Joints5 out;
    for (int i = 0; i < dof_waist; ++i)
        out(i) = kMotorDirect[i] * q(i);
    return out;
}

void hold_roll_yaw_zero(LowerBody::Joints5 &q)
{
    q(LowerBody::WaistRoll) = 0.0;
    q(LowerBody::WaistYaw) = 0.0;
}

void unwrap_toward(LowerBody::Joints5 &q, const LowerBody::Joints5 &seed)
{
    for (int i = 0; i < dof_waist; ++i)
        q(i) = seed(i) + LowerBody::WrapPi(q(i) - seed(i));
}
} // namespace

WaistRobot::WaistRobot()
{
    // 运控软限位；笛卡尔可达仍以 LowerBody::Reachable 为准。
    Matrix<double, 1, dof_waist> q_min, q_max;
    q_min << -120, -180, -180, -45, -180;
    q_max << 120, 180, 180, 45, 180;
    q_min *= rad;
    q_max *= rad;
    limit.row(0) = q_min;
    limit.row(1) = q_max;
    tool << 0, 0, 0, 0, 0, 0;

    const auto &p = kin_.params();
    std::cout << std::fixed << std::setprecision(3)
              << "[waist] LowerBody L1=" << p.L1 << " L2=" << p.L2 << " d2=" << p.d2
              << " m，站立 TCP z=" << (p.L1 + p.L2)
              << "（电机零位=直立；踝/膝相对电机取反，笛卡尔锁姿态=直立 pitch/roll/yaw=0）\n";
}

bool WaistRobot::jointsInLimit(const Joints5 &q) const
{
    for (int i = 0; i < dof_waist; ++i)
    {
        if (q(i) < limit(0, i) - 1e-3 || q(i) > limit(1, i) + 1e-3)
            return false;
    }
    return true;
}

void WaistRobot::logJoints(const char *tag, const Joints5 &q) const
{
    LowerBody::Joints3 q_leg, q_w;
    kin_.Split(q, q_leg, q_w);
    const Joints5 q_mot = apply_motor_direct(q);
    const Eigen::Matrix4d T = kin_.Forward_Kinematics(q);
    std::cout << std::fixed << std::setprecision(1)
              << "[waist] " << tag << " 几何deg 踝/膝/髋/滚/回=["
              << (q(0) / rad) << "," << (q(1) / rad) << "," << (q(2) / rad) << ","
              << (q(3) / rad) << "," << (q(4) / rad) << "] 电机deg ID4/3/2/5/1=["
              << (q_mot(0) / rad) << "," << (q_mot(1) / rad) << "," << (q_mot(2) / rad) << ","
              << (q_mot(3) / rad) << "," << (q_mot(4) / rad) << "] pitch=" << (q_w(0) / rad)
              << "° tcp=(" << std::setprecision(4) << T(0, 3) << "," << T(1, 3) << ","
              << T(2, 3) << ")\n";
}

Matrix<double, 1, 6> WaistRobot::getTool()
{
    return TR(tool);
}

WaistRobot::Joints5 WaistRobot::readJoints()
{
    int data[dof_waist] = {};
    get_motor_waist_position(data);
    Joints5 q_mot;
    for (int i = 0; i < dof_waist; ++i)
        q_mot(i) = static_cast<double>(data[i]) / rad2cnt;
    return apply_motor_direct(q_mot);
}

MatrixXd WaistRobot::getJointPos()
{
    return readJoints();
}

MatrixXd WaistRobot::getTcpPos()
{
    return T2PosEulerAngles(fk(readJoints()));
}

Matrix4d WaistRobot::fk(const Joints5 &q)
{
    return kin_.Forward_Kinematics(q);
}

int WaistRobot::servoJ(const Joints5 &qd)
{
    const Joints5 q_mot = apply_motor_direct(qd);
    int motorPositions[dof_waist];
    for (int i = 0; i < dof_waist; ++i)
        motorPositions[i] = static_cast<int>(q_mot(i) * rad2cnt);
    set_waist_motor_position(motorPositions);
    return 0;
}

int WaistRobot::ik(Matrix<double, 1, 6> &pos, Joints5 &q)
{
    return ik(TR(pos), q);
}

int WaistRobot::ik(const Eigen::Matrix4d &T_base_tcp, Joints5 &q)
{
    const Joints5 seed = q;
    Joints5 q_ik = kin_.Inverse_Kinematics(T_base_tcp);
    if (!LowerBody::Reachable(q_ik))
        return -2;
    unwrap_toward(q_ik, seed);
    if (!jointsInLimit(q_ik))
        return 1;
    q = q_ik;
    return 0;
}

int WaistRobot::moveLToPos(Matrix<double, 1, 6> &posd, double vel)
{
    return moveLToPos(posd, vel, nullptr);
}

int WaistRobot::moveLInterp(
    const Eigen::Matrix4d &T1,
    const Eigen::Matrix4d &T2,
    double vel,
    const std::function<bool()> &should_abort)
{
    auto aborted = [&]() -> bool {
        return can_io_faulted() || hardware_abort_requested() ||
               (static_cast<bool>(should_abort) && should_abort());
    };
    if (aborted())
        return -3;

    Joints5 qc = readJoints();
    Joints5 q_goal = qc;
    if (ik(T2, q_goal) != 0)
    {
        std::cout << "Move_Limit\n";
        logJoints("目标不可达，当前", qc);
        return -2;
    }
    hold_roll_yaw_zero(q_goal);

    Matrix<double, 1, 6> axis_p1, axis_p2;
    axis_p1.setZero();
    axis_p2 = T2Axispos(T1.inverse() * T2);
    const double Tf = calculateTf(axis_p1, axis_p2, vel);
    const double dt = 5e-3;

    std::cout << std::fixed << std::setprecision(4)
              << "[waist] 笛卡尔直线 xyz (" << T1(0, 3) << "," << T1(1, 3) << "," << T1(2, 3)
              << ") → (" << T2(0, 3) << "," << T2(1, 3) << "," << T2(2, 3)
              << ") 目标姿态=直立 Tf=" << Tf << " s\n";
    logJoints("起点", qc);

    double t = 0.0;
    while (t < Tf)
    {
        if (aborted())
        {
            std::cout << "[waist] 运动中止，保持当前指令位置\n";
            return -3;
        }
        auto cycle_start = chrono::high_resolution_clock::now();
        auto [axis_pos, daxis_pos, ddaxis_pos] = quinticInterp(t, Tf, axis_p1, axis_p2);
        const Eigen::Matrix4d Td = T1 * Axispos2T(axis_pos);
        if (ik(Td, qc) != 0)
        {
            std::cout << "Move_error\n";
            return -1;
        }
        hold_roll_yaw_zero(qc);
        servoJ(qc);
        const double elapsed_ms =
            duration<double, milli>(chrono::high_resolution_clock::now() - cycle_start).count();
        const double sleep_ms = std::max(0.0, dt * 1000.0 - elapsed_ms);
        if (sleep_ms > 1e-6)
            this_thread::sleep_for(duration<double, milli>(sleep_ms));
        t += dt;
    }
    servoJ(q_goal);
    usleep(50000);
    logJoints("到位目标", q_goal);
    logJoints("到位回读", readJoints());
    return 0;
}

int WaistRobot::moveLToPos(Matrix<double, 1, 6> &posd, double vel, const std::function<bool()> &should_abort)
{
    enable_waist_motors();
    const Joints5 qc = readJoints();
    const Eigen::Matrix4d T1 = fk(qc);
    // 目标姿态固定直立。以前复制当前 FK 旋转，点头（髋电机 CAN2）会一直带着走。
    Eigen::Matrix4d T2 = Eigen::Matrix4d::Identity();
    T2(0, 3) = posd(0);
    T2(1, 3) = posd(1);
    T2(2, 3) = posd(2);
    return moveLInterp(T1, T2, vel, should_abort);
}

int WaistRobot::moveLToPosZ(double z_target, double vel)
{
    return moveLToPosZ(z_target, vel, nullptr);
}

int WaistRobot::moveLToPosZ(double z_target, double vel, const std::function<bool()> &should_abort)
{
    auto aborted = [&]() -> bool {
        return can_io_faulted() || hardware_abort_requested() ||
               (static_cast<bool>(should_abort) && should_abort());
    };
    if (aborted())
        return -3;

    enable_waist_motors();
    const Joints5 qc = readJoints();
    const Eigen::Matrix4d T1 = fk(qc);
    Eigen::Matrix4d T2 = Eigen::Matrix4d::Identity();
    T2(0, 3) = T1(0, 3);
    T2(1, 3) = T1(1, 3);
    T2(2, 3) = z_target;
    std::cout << std::fixed << std::setprecision(4)
              << "[waist] 竖直 Z 锁 x=" << T1(0, 3) << " y=" << T1(1, 3)
              << " 姿态=直立  " << T1(2, 3) << " → " << z_target << " m\n";
    return moveLInterp(T1, T2, vel, should_abort);
}

int WaistRobot::moveJToJoint(Joints5 &qd, double vel)
{
    const Joints5 qc = readJoints();
    const Eigen::Matrix4d T1 = fk(qc);
    const Eigen::Matrix4d T2 = fk(qd);
    const double Tf = calculateTf(T2Axispos(T1), T2Axispos(T2), vel);
    return moveJToJoint_Tf(qd, Tf);
}

int WaistRobot::moveJToJoint_Tf(Joints5 &q_end, double Tf)
{
    Joints5 q_start = readJoints();
    unwrap_toward(q_end, q_start);
    double t = 0.0;
    const double dt = 5e-3;
    while (t < Tf)
    {
        auto cycle_start = chrono::high_resolution_clock::now();
        auto [q, dq, ddq] = quinticInterp(t, Tf, q_start, q_end);
        Joints5 qj = q;
        servoJ(qj);
        if (hardware_abort_requested() || can_io_faulted())
        {
            std::cout << "[waist] STOP/CAN，停止关节运动\n";
            return -2;
        }
        const double elapsed_ms =
            duration<double, std::milli>(chrono::high_resolution_clock::now() - cycle_start).count();
        const double sleep_ms = std::max(0.0, dt * 1000.0 - elapsed_ms);
        if (sleep_ms > 1e-6)
            std::this_thread::sleep_for(duration<double, std::milli>(sleep_ms));
        t += dt;
    }
    servoJ(q_end);
    return 0;
}

int WaistRobot::jogJoint(int joint_1based, double dq_rad, double vel)
{
    if (joint_1based < 1 || joint_1based > dof_waist)
    {
        std::cerr << "[waist] jog joint 必须是 1..5（脚踝/膝盖/髋/侧倾/回转）\n";
        return -1;
    }
    if (!std::isfinite(dq_rad) || std::abs(dq_rad) < 1e-4)
    {
        std::cerr << "[waist] jog dq_rad 太小\n";
        return -1;
    }
    if (hardware_abort_requested())
        return -3;

    enable_waist_motors();
    Joints5 q = readJoints();
    const int i = joint_1based - 1;
    const double q0 = q(i);
    const double q1 = std::clamp(q0 + dq_rad, limit(0, i), limit(1, i));
    if (std::abs(q1 - q0) < 1e-4)
    {
        std::cerr << std::fixed << std::setprecision(2)
                  << "[waist] jog " << kJointName[i] << " 已在限位 "
                  << (q0 / rad) << " deg，未动\n";
        return -2;
    }
    q(i) = q1;
    std::cout << std::fixed << std::setprecision(2)
              << "[waist] jog " << kJointName[i] << " "
              << (q0 / rad) << " → " << (q1 / rad) << " deg  (Δ "
              << ((q1 - q0) / rad) << " deg)，关节插补\n";
    if (vel < 1e-3)
        vel = 0.08;
    const int rc = moveJToJoint(q, vel);
    logJoints("jog后", readJoints());
    return rc;
}

int WaistRobot::moveYawAbsDeg(double yaw_deg, double vel)
{
    if (!std::isfinite(yaw_deg))
    {
        std::cerr << "[waist] yaw 无效\n";
        return -1;
    }
    if (hardware_abort_requested())
        return -3;

    enable_waist_motors();
    Joints5 q = readJoints();
    const double q0 = q(LowerBody::WaistYaw);
    const double q1 = std::clamp(yaw_deg * rad, limit(0, LowerBody::WaistYaw), limit(1, LowerBody::WaistYaw));
    if (std::abs(q1 - q0) <= 1e-3)
    {
        std::cout << std::fixed << std::setprecision(2)
                  << "[waist] 回转已在 " << (q0 / rad) << " deg，跳过\n";
        return 0;
    }
    q(LowerBody::WaistYaw) = q1;
    if (vel < 1e-3)
        vel = 0.15;
    std::cout << std::fixed << std::setprecision(2)
              << "[waist] 回转 " << (q0 / rad) << " → " << (q1 / rad)
              << " deg（+Z 向左为正），只动 yaw\n";
    const int rc = moveJToJoint(q, vel);
    logJoints("回转后", readJoints());
    return rc;
}

int WaistRobot::moveJToPos(Matrix<double, 1, 6> &posd, double vel)
{
    Joints5 qd = readJoints();
    const int ret = ik(posd, qd);
    if (ret != 0)
        return ret;
    return moveJToJoint(qd, vel);
}
