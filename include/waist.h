#ifndef _WAIST_H_
#define _WAIST_H_

#include "head.h"
#include <functional>
using namespace Eigen;
using namespace std::chrono;
using namespace std;


constexpr int dof_3 = 3;

// 腰部三轴机器人（与 ArmRobot 双臂区分）
class WaistRobot
{
private:
    const double a2 = 300e-3;
    const double a3 = 422.5e-3;

    double res = 65536;
    double ratio = 4;
    // double ratio = 101;

    Matrix<double, 1, 6> tool;
    Matrix<double, 1, dof_3> w;
    Matrix<double, 2, dof_3> limit;
    Matrix<double, 1, dof_3> init_qref, q2motor_direct, q2motor_offset;
    Matrix<double, 4, 4> T_falan_tcp, T_falan_tcp_Inv;
    int deviceInd, canInd;
    double rad2cnt = (ratio * res) / (2 * M_PI); 
    double vel2hz = (ratio * 1000) / (2 * M_PI);
    double Torque2Current = (ratio * 100) / (2 * M_PI);
    uint8_t MotorsIDlist[dof_3];

    Matrix<double, dof_3, 4> MDH;
    int select = 0;
    

public:
    // 构造函数，用于初始化机器人参数
    WaistRobot()
    {
        Matrix<double, 1, dof_3> q_limit_min, q_limit_max;
        Matrix<double, dof_3, 1> theta, d, a, alpha, offset;

        alpha << M_PI / 2, 0, 0;
        a << 0, a2, a3;
        d << 0, 0, 0;
        offset << 0, 0, 0;
        MDH << alpha, a, d, offset;

        q_limit_min << 0, -180, -180;
        q_limit_max << 107, 180, 180;
        q_limit_min *= rad;
        q_limit_max *= rad;
        limit.row(0) = q_limit_min;
        limit.row(1) = q_limit_max;
        select = 1;
        tool << 0e-3, 0e-3, 0, 0, 0, 0;
        q2motor_direct << -1, -1, 1;
        // q2motor_offset << 0, M_PI / 2, -M_PI / 2, 0, M_PI / 2, 0, 0; // 电机零位向模型零位转动
        q2motor_offset << M_PI / 2, 0, 0; // 电机零位向模型零位转动
        MotorsIDlist[0] = 4;
        MotorsIDlist[1] = 3;
        MotorsIDlist[2] = 2;
    }

    Matrix<double, 1, 6> getTool();
    MatrixXd q2MotorAngle(const Matrix<double, 1, dof_3> &q);
    MatrixXd MotorAngle2q(const Matrix<double, 1, dof_3> &MotorAngle);
    MatrixXd J_tcp_crossproduct(const Matrix<double, 1, dof_3> &q);
    Matrix4d fk(const Matrix<double, 1, dof_3> &q);
    int ik(Matrix<double, 1, 6> &pos, Matrix<double, 1, dof_3> &q);
    int servoJ(const Matrix<double, 1, dof_3> &qd);
    int speedJ(Matrix<double, 1, dof_3> &dq);
    int tauJ(Matrix<double, 1, dof_3> &tau);

    int moveJToJoint_Tf(Matrix<double, 1, dof_3> &q_end, double Tf);
    int moveJToJoint(Matrix<double, 1, dof_3> &qd, double vel);
    int moveJToPos(Matrix<double, 1, 6> &posd, double vel);
    /** 腰 TCP 笛卡尔直线；0 成功，-1 中途 IK 失败，-2 起点超限，-3 外部中止 */
    int moveLToPos(Matrix<double, 1, 6> &posd, double vel);
    int moveLToPos(Matrix<double, 1, 6> &posd, double vel, const std::function<bool()> &should_abort);

    //  *********************Get********************
    MatrixXd getCsp();
    MatrixXd getJointPos();
    MatrixXd getTcpPos();

    void getJointState(Matrix<double, 1, dof_3> &qc, Matrix<double, 1, dof_3> &dqc);
};

#endif