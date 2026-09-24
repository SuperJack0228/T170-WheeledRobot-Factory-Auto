#ifndef _WAIST_H_
#define _WAIST_H_

#include "head.h"
#include "LowerBody.h"

#include <functional>

using namespace Eigen;
using namespace std::chrono;
using namespace std;

constexpr int dof_waist = LowerBody::Dof;

/** 腰腿 5 轴：LowerBody 几何角 [脚踝, 膝盖, 髋, 侧倾, 回转]，CAN ID 4/3/2/5/1。
 *  读写时踝/膝相对电机取反（现场正向前，库正往后坐）；髋/侧倾/回转同号。 */
class WaistRobot
{
private:
    LowerBody kin_;
    double res = 65536;
    double ratio = 4;
    double rad2cnt = (ratio * res) / (2.0 * M_PI);
    double vel2hz = (ratio * 1000) / (2.0 * M_PI);
    Matrix<double, 2, dof_waist> limit;
    Matrix<double, 1, 6> tool;

    int moveLInterp(
        const Eigen::Matrix4d &T1,
        const Eigen::Matrix4d &T2,
        double vel,
        const std::function<bool()> &should_abort);
    void logJoints(const char *tag, const LowerBody::Joints5 &q) const;
    bool jointsInLimit(const LowerBody::Joints5 &q) const;
    LowerBody::Joints5 readJoints();

public:
    WaistRobot();

    using Joints5 = LowerBody::Joints5;

    Matrix<double, 1, 6> getTool();
    Matrix4d fk(const Joints5 &q);
    int ik(Matrix<double, 1, 6> &pos, Joints5 &q);
    /** 4×4 → LowerBody 逆解。q 为初值，用于把解展开到最近周期。 */
    int ik(const Eigen::Matrix4d &T_base_tcp, Joints5 &q);
    int servoJ(const Joints5 &qd);

    int moveJToJoint_Tf(Joints5 &q_end, double Tf);
    int moveJToJoint(Joints5 &qd, double vel);
    /** 关节点动。1脚踝 2膝盖 3髋 4侧倾 5回转，dq 为几何弧度。 */
    int jogJoint(int joint_1based, double dq_rad, double vel = 0.08);
    /** 只改回转角（几何 deg，+Z 向左为正）。不走笛卡尔，避免把 yaw 锁回 0。 */
    int moveYawAbsDeg(double yaw_deg, double vel = 0.15);
    int moveJToPos(Matrix<double, 1, 6> &posd, double vel);
    /** 腰 TCP 笛卡尔直线，目标姿态固定直立（不复制当前点头）。0 成功，-1 中途 IK 失败，-2 目标不可达，-3 外部中止 */
    int moveLToPos(Matrix<double, 1, 6> &posd, double vel);
    int moveLToPos(Matrix<double, 1, 6> &posd, double vel, const std::function<bool()> &should_abort);
    /** 只动 Z，X/Y 锁当前，姿态拉回直立。home / 抓取升降用这个。 */
    int moveLToPosZ(double z_target, double vel);
    int moveLToPosZ(double z_target, double vel, const std::function<bool()> &should_abort);

    MatrixXd getJointPos();
    MatrixXd getTcpPos();
};

#endif
