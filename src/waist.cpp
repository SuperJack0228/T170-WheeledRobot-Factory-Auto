#include "waist.h"
#include "Ti5_socketcan.h"

// 注意：所有函数实现前的 Robot:: 都要改为 WaistRobot::

Matrix<double, 1, 6> WaistRobot::getTool()
{
    return TR(tool);
}

MatrixXd WaistRobot::q2MotorAngle(const Matrix<double, 1, dof_3> &q)
{
    Matrix<double, 1, dof_3> MotorAngle;
    MotorAngle = q.cwiseProduct(q2motor_direct) + q2motor_offset;
    return MotorAngle;
}

MatrixXd WaistRobot::MotorAngle2q(const Matrix<double, 1, dof_3> &MotorAngle)
{
    Matrix<double, 1, dof_3> q;
    q = (MotorAngle - q2motor_offset).cwiseQuotient(q2motor_direct);
    return q;
}

MatrixXd WaistRobot::getJointPos()
{
    int dof_3 = 3; // 假设这是变量
    MatrixXd motor_q(1, dof_3), q(1, dof_3), motor_pos(1, dof_3);
    MatrixXd dataList(1, dof_3);
    int data[3] = {};

    get_motor_waist_position(data);

    // 使用循环赋值
    for (int i = 0; i < dof_3; i++)
    {
        dataList(i) = static_cast<double>(data[i]);
    }

    // dataList = getCSP(0, MotorsIDlist, dof_3);

  //  motor_pos = dataList.block(2, 0, 1, dof_3);
    motor_pos = dataList / rad2cnt;
    q = MotorAngle2q(motor_pos);

    return q;
}

MatrixXd WaistRobot::getCsp()
{
    Matrix<double, 1, dof_3> motor_current, motor_vel, motor_pos, q, dq, tor;

    int32_t MotorPosition[dof_3];
    Matrix<double, 3, dof_3> dataList, output;

    // auto cycle_start = chrono::high_resolution_clock::now();
    // dataList = getCSP(0, MotorsIDlist, dof_3);
    motor_current = dataList.block(0, 0, 1, dof_3);
    motor_vel = dataList.block(1, 0, 1, dof_3);
    motor_pos = dataList.block(2, 0, 1, dof_3);

    motor_pos = motor_pos / rad2cnt;
    motor_vel = motor_vel / vel2hz;
    q = MotorAngle2q(motor_pos);
    dq = motor_vel.cwiseQuotient(q2motor_direct);
    tor = motor_current.cwiseQuotient(q2motor_direct);
    output.row(0) = q;
    output.row(1) = dq;
    output.row(2) = tor;
    return output;
}

// 雅可比矩阵计算函数-矢量积法
MatrixXd WaistRobot::J_tcp_crossproduct(const Matrix<double, 1, dof_3> &q)
{
    VectorXd alp = MDH.col(0);
    VectorXd a = MDH.col(1);
    VectorXd d = MDH.col(2);
    VectorXd offset = MDH.col(3);
    VectorXd q_offset = q.transpose() + offset;
    Matrix4d T = Matrix4d::Identity();

    for (int i = 0; i < dof_3; ++i)
    {
        Matrix4d T_i = MDHTrans(alp(i), a(i), d(i), q_offset(i));
        T = T * T_i;
    }

    Matrix4d Tend = TR(tool);
    Matrix4d T_b_end = T * Tend;
    Matrix4d T1 = fk(q);
    Vector3d p_ee = T1.block<3, 1>(0, 3);

    MatrixXd J(6, dof_3);
    J.setZero();
    T = Matrix4d::Identity();
    for (int i = 0; i < dof_3; ++i)
    {
        Matrix4d T_i = MDHTrans(alp(i), a(i), d(i), q_offset(i));
        T = T * T_i;
        Vector3d z_i = T.block<3, 1>(0, 2);
        Vector3d p_i = T.block<3, 1>(0, 3);
        J.block<3, 1>(0, i) = z_i.cross(p_ee - p_i);
        J.block<3, 1>(3, i) = z_i;
    }

    MatrixXd transform(6, 6);
    transform.setZero();
    transform.block<3, 3>(0, 0) = T_b_end.block<3, 3>(0, 0);
    transform.block<3, 3>(3, 3) = T_b_end.block<3, 3>(0, 0);

    return transform.transpose() * J;
}

MatrixXd WaistRobot::getTcpPos()
{
    Matrix<double, 1, dof_3> motor_q, q;
    Matrix<double, 4, 4> T;
    Matrix<double, 1, 6> pos;
    q = getJointPos();

    T = fk(q);
    pos = T2PosEulerAngles(T);
    return pos;
}

// 正向运动学函数
Matrix4d WaistRobot::fk(const Matrix<double, 1, dof_3> &q)
{
    int n = dof_3;
    Matrix4d T = Matrix4d::Identity();
    VectorXd alp = MDH.col(0);
    VectorXd a = MDH.col(1);
    VectorXd d = MDH.col(2);
    VectorXd offset = MDH.col(3);

    for (int i = 0; i < n; ++i)
    {
        Matrix4d T_i = MDHTrans(alp(i), a(i), d(i), q(i) + offset(i));
        T = T * T_i;
    }
    Matrix4d T_tcp = T * TR(tool);
    return T_tcp;
}

// 核心函数：输入关节弧度值，无阻塞批量发送到电机
int WaistRobot::servoJ(const Matrix<double, 1, dof_3> &qd)
{
    Matrix<double, 1, dof_3> motor_q;
    motor_q = q2MotorAngle(qd);

    int32_t motorPositions[dof_3];
    for (int i = 0; i < dof_3; ++i)
    {
        motorPositions[i] = static_cast<int32_t>(motor_q(i) * rad2cnt);
    }

    set_waist_motor_position(motorPositions);
    return 0;
}

// 核心函数：输入关节弧度值，无阻塞批量发送到电机
int WaistRobot::speedJ(Matrix<double, 1, dof_3> &dq)
{
    Matrix<double, 1, dof_3> motor_dq;
    motor_dq = dq.cwiseProduct(q2motor_direct);

    int32_t motorSpeeds[dof_3];
    for (int i = 0; i < dof_3; ++i)
    {
        motorSpeeds[i] = static_cast<int32_t>(motor_dq(i) * vel2hz);
    }

    //  setSpeeds(0, MotorsIDlist, motorSpeeds);
    return 0;
}

// 核心函数：输入关节弧度值，无阻塞批量发送到电机
int WaistRobot::tauJ(Matrix<double, 1, dof_3> &tau)
{
    int32_t motorCurrents[dof_3];
    for (int i = 0; i < dof_3; ++i)
    {
        motorCurrents[i] = static_cast<int32_t>(tau(i) * q2motor_direct(i));
    }

    // setCurrents(0, MotorsIDlist, motorCurrents);
    return 0;
}

int WaistRobot::moveLToPos(Matrix<double, 1, 6> &posd, double vel)
{
    return moveLToPos(posd, vel, nullptr);
}

int WaistRobot::moveLToPos(Matrix<double, 1, 6> &posd, double vel, const std::function<bool()> &should_abort)
{
    Matrix<double, 1, 6> pos2, axis_p1, axis_p2;
    Matrix<double, 1, dof_3> qc, q_ref;
    Matrix<double, 4, 4> T1, T2, Td, T12;

    auto aborted = [&]() -> bool {
        return can_io_faulted() || hardware_abort_requested() ||
               (static_cast<bool>(should_abort) && should_abort());
    };

    if (aborted())
        return -3;

    qc = getJointPos();
    q_ref = qc;
    int ret = ik(posd, q_ref);

    if (ret != 0)
    {
        cout << "Move_Limit" << endl;
        return -2;
    }

    T1 = fk(qc);
    pos2 = posd;
    T2 = TR(pos2);
    T12 = T1.inverse() * T2;
    axis_p1.setZero();
    axis_p2 = T2Axispos(T12);

    const double Tf = calculateTf(axis_p1, axis_p2, vel);
    const double dt = 5e-3;
    double t = 0.0;

    while (t < Tf)
    {
        if (aborted())
        {
            cout << "[waist] 运动中止，保持当前指令位置" << endl;
            return -3;
        }

        auto cycle_start = chrono::high_resolution_clock::now();
        auto [axis_pos, daxis_pos, ddaxis_pos] = quinticInterp(t, Tf, axis_p1, axis_p2);

        Td = T1 * Axispos2T(axis_pos);
        posd = T2PosEulerAngles(Td);
        int ret1 = ik(posd, qc);
        if (ret1 != 0)
        {
            cout << "Move_error" << endl;
            return -1;
        }
        servoJ(qc);

        double elapsed_ms = duration<double, milli>(chrono::high_resolution_clock::now() - cycle_start).count();
        double sleep_ms = max(0.0, dt * 1000 - elapsed_ms);
        if (sleep_ms > 1e-6)
            this_thread::sleep_for(duration<double, milli>(sleep_ms));

        t = t + dt;
    }

    return 0;
}

int WaistRobot::moveJToJoint(Matrix<double, 1, dof_3> &qd, double vel)
{
    Matrix<double, 4, 4> T1, T2;
    Matrix<double, 1, 6> axis_p1, axis_p2;
    Matrix<double, 1, dof_3> qc;
    qc = getJointPos();
    T1 = fk(qc);
    T2 = fk(qd);

    axis_p1 = T2Axispos(T1);
    axis_p2 = T2Axispos(T2);
    double Tf = calculateTf(axis_p1, axis_p2, vel);
    int ret = moveJToJoint_Tf(qd, Tf);
    return ret;
}

int WaistRobot::moveJToJoint_Tf(Matrix<double, 1, dof_3> &q_end, double Tf)
{
    Matrix<double, 1, dof_3> q_start, qc, q, dq, dqc;
    double t, dt;
    q_start = getJointPos();
    t = 0;
    dt = 5e-3;
    auto cycle_start = chrono::high_resolution_clock::now();

    while (t < Tf)
    {
        cycle_start = chrono::high_resolution_clock::now();
        auto [q, dq, ddq] = quinticInterp(t, Tf, q_start, q_end);
        servoJ(q);
        if (hardware_abort_requested() || can_io_faulted())
        {
            cout << "[waist] STOP/CAN，停止关节运动" << endl;
            return -2;
        }

        double elapsed_ms = duration<double, std::milli>(chrono::high_resolution_clock::now() - cycle_start).count();

        double sleep_ms = max(0.0, dt * 1000 - elapsed_ms);
        if (sleep_ms > 1e-6)
        {
            auto sleep_duration = duration<double, std::milli>(sleep_ms);
            std::this_thread::sleep_for(sleep_duration);
        }

        t = t + dt;
    }
    return 0;
}

int WaistRobot::moveJToPos(Matrix<double, 1, 6> &posd, double vel)
{
    Matrix<double, 1, dof_3> qc, qd;
    Matrix<double, 4, 4> T1, T2;
    Matrix<double, 1, 6> axis_p1, axis_p2;
    qc = getJointPos();
    qd = qc;
    int ret = ik(posd, qd);

    T1 = fk(qc);
    T2 = TR(posd);
    axis_p1 = T2Axispos(T1);
    axis_p2 = T2Axispos(T2);
    double Tf = calculateTf(axis_p1, axis_p2, vel);
    int ret1 = moveJToJoint_Tf(qd, Tf);

    return ret1;
}

void WaistRobot::getJointState(Matrix<double, 1, dof_3> &qc, Matrix<double, 1, dof_3> &dqc)
{
    // // auto data = getCsp(); // 假设t5是内部关节状态接口
    // qc = data.row(0);  // 当前关节位置
    // dqc = data.row(1); // 当前关节速度
}

int WaistRobot::ik(Matrix<double, 1, 6> &pos, Matrix<double, 1, dof_3> &q)
{
    const double eps = 1e-6;
    const double singular_threshold = 1 * M_PI / 180; // 对应MATLAB的1*tk.rad，约0.017rad

    // -------------------------- 1. 初始化与参数提取 --------------------------
    Eigen::MatrixXd solutions(0, 3); // 候选解集合（每行1个3关节解）

    // 提取机器人参数（MDH为3×4矩阵，列0:alpha, 列1:a, 列2:d, 列3:offset）
    const MatrixXd alpha = MDH.col(0);  // 3×1
    const MatrixXd a = MDH.col(1);      // 3×1
    const double a2 = a(1, 0);          // 对应MATLAB a2=a(2)（注意索引从0开始）
    const double a3 = a(2, 0);          // 对应MATLAB a3=a(3)
    const MatrixXd offset = MDH.col(3); // 3×1（关节偏移角）

    // 计算变换矩阵（基→末端法兰，去除工具坐标系影响）
    Matrix4d T_falan_tcp = TR(tool);
    Matrix4d T_base_tcp = TR(pos);
    Matrix4d T_base_falan = T_base_tcp * T_falan_tcp.inverse();
    Matrix4d Tend = T_base_falan;

    // 提取末端变换矩阵关键参数（位置+姿态）
    const double nx = Tend(0, 0);
    const double nz = Tend(2, 0);
    const double px = Tend(0, 3);
    const double pz = Tend(2, 3);

    // -------------------------- 2. 工作空间检查（末端是否接近原点） --------------------------
    const double p_sq = px * px + pz * pz;
    if (p_sq < eps)
    {
        return -1; // 末端位置接近原点，无法求解
    }

    // -------------------------- 3. 求解t2（核心关节，两个候选解） --------------------------
    double cos_t2 = (p_sq - a2 * a2 - a3 * a3) / (2 * a2 * a3);
    cos_t2 = clamp(cos_t2, -1.0, 1.0); // 数值保护，避免acos参数超范围

    const double t2_sol1 = acos(cos_t2); // 解1：肘上构型
    const double t2_sol2 = -t2_sol1;     // 解2：肘下构型
    const vector<double> t2_sols = {t2_sol1, t2_sol2};

    if (abs(t2_sol1) <= singular_threshold)
    {
        cout << "workapce_limit" << "\n";
        return -2;
    }

    // -------------------------- 4. 遍历t2解，求解t1和t3 --------------------------
    for (double t2 : t2_sols)
    {
        // 计算中间变量K和L（对应MATLAB逻辑）
        const double K = a2 + a3 * cos(t2);
        const double L = a3 * sin(t2);

        // 求解t1（四象限反正切，避免除零）
        const double num_t1 = pz * K - px * L;
        const double den_t1 = px * K + pz * L;
        const double t1 = atan2(num_t1, den_t1);

        // 求解总旋转角theta（基于末端姿态）
        const double theta = atan2(nz, nx);

        // 求解t3（theta = t1 + t2 + t3 → t3 = theta - t1 - t2）
        double t3 = theta - t1 - t2;

        // 角度归一化到[-π, π]
        t3 = fmod(t3 + M_PI, 2 * M_PI) - M_PI;

        // -------------------------- 5. 生成候选解并添加到集合 --------------------------
        MatrixXd q_candidate(1, 3);
        q_candidate << t1, t2, t3;

        // 候选解角度归一化（确保统一范围）
        for (int i = 0; i < dof_3; ++i)
        {
            q_candidate(0, i) = fmod(q_candidate(0, i) + M_PI, 2 * M_PI) - M_PI;
        }

        // 动态扩展候选解集合
        solutions.conservativeResize(solutions.rows() + 1, 3);
        solutions.row(solutions.rows() - 1) = q_candidate;
    }

    // -------------------------- 6. 检查是否有有效候选解 --------------------------
    if (solutions.rows() == 0)
    {
        return -3; // 所有解均奇异，位置无解
    }

    const MatrixXd q_best = solutions.row(0);

    // -------------------------- 8. 限位检查 --------------------------
    bool in_limit = true;
    for (int i = 0; i < dof_3; ++i)
    {
        if (q_best(0, i) < limit(0, i) - eps || q_best(0, i) > limit(1, i) + eps)
        {
            in_limit = false;
            break;
        }
    }

    // -------------------------- 9. 输出结果与返回状态 --------------------------
    if (in_limit)
    {
        q = q_best; // 输出最优解（1×3矩阵，适配dof_3=3）
        return 0;   // 成功：有解且在限位内
    }
    else
    {
        return 1; // 有解但超限位
    }
}