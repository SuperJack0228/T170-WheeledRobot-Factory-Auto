#ifndef TI5_ROBOTICS_H
#define TI5_ROBOTICS_H

#include <Eigen/Dense>

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

/// Ti5 机械臂算法 SDK。
///
/// 单位：关节位置 / 角速度 — rad / rad/s；
/// MIT 指令 — 目标角 rad、目标速度 rad/s，Kp 为 N·m/rad，Kd 为 N·m/(rad/s)，前馈力矩 N·m；
/// 关节力矩传感器 — N·m；负载 — kg、m；
/// 笛卡尔位姿 — `Eigen::Matrix4d`（TCP 相对基座齐次变换）。
/// 构造：`model_key` 为 yaml 机型键；七轴 / A6 传 `variant`（如 left / right）。本库只读 yaml，不写回。
/// A6 动力学接口在非 A6 机型：返回向量的得到空向量，零重力与阻抗把 commands 清空；返回 int 的接口返回 -1。
/// `Joint_Acceleration` 与机型无关。
class Ti5_Robotics
{
public:
    /// 单轴一拍 MIT 指令。库只算这些数，不切电机模式。
    struct Mit_Command
    {
        double q = 0;   ///< 目标角 (rad)
        double dq = 0;  ///< 目标速度 (rad/s)
        double kp = 0;  ///< N·m/rad
        double kd = 0;  ///< N·m/(rad/s)
        double tau = 0; ///< 前馈力矩 (N·m)
    };

    /// @param model_key yaml 机型键（如 "A6_170"）
    /// @param variant 子配置（A6/T7 的 left/right；其它可空）
    /// @throws std::runtime_error 机型或 variant 不存在
    explicit Ti5_Robotics(const std::string &model_key, const std::string &variant = "");

    ~Ti5_Robotics();
    Ti5_Robotics(Ti5_Robotics &&) noexcept;
    Ti5_Robotics &operator=(Ti5_Robotics &&) noexcept;
    Ti5_Robotics(const Ti5_Robotics &) = delete;
    Ti5_Robotics &operator=(const Ti5_Robotics &) = delete;

    /// 笛卡尔直线规划线速度上限 (m/s)
    void Set_Cartesian_Linear_Velocity(double velocity);

    /// 写入改进 DH。只更新内存，不写 yaml。正运动学、雅可比和 A6 动力学都用这份参数。
    /// A6 解析逆解仍用构造时读入的 DH。这份和构造时不同，逆解会在解析后再锁住 J7 抛光。
    /// 行数为轴数，下标 0 为 J1。列依次为 alpha (rad)、a (m)、d (m)、offset (rad)，与 yaml 的 alpha、a、d、offset 相同。
    /// 行数或列数不对、或含非有限数时抛 std::invalid_argument。
    void Set_DH_Para(const Eigen::Ref<const Eigen::MatrixXd> &dh_para);

    /// 当前改进 DH，dof×4。列依次为 alpha (rad)、a (m)、d (m)、offset (rad)。
    Eigen::MatrixXd DH_Para() const;

    /// 写入法兰到 TCP 的工具位姿。只更新内存，不写 yaml。正逆解和 A6 动力学都用这份参数。
    /// 六个数为 [x, y, z, rx, ry, rz]，与 yaml 的 tool 相同。平移为 m，姿态为 ZYX 欧拉角 (rad)。
    /// 长度不是 6 或含非有限数时抛 std::invalid_argument。
    void Set_Tool(const Eigen::Ref<const Eigen::RowVectorXd> &tool);

    /// 当前工具位姿 [x, y, z, rx, ry, rz]，单位 m、rad。
    Eigen::RowVectorXd Tool() const;

    /// 正运动学：基座 → TCP
    Eigen::Matrix4d Forward_Kinematics(const Eigen::Ref<const Eigen::RowVectorXd> &joint_positions) const;

    /// 正运动学：基座 → 法兰
    Eigen::Matrix4d Flange_Transform(const Eigen::Ref<const Eigen::RowVectorXd> &joint_positions) const;

    /// 逆运动学。A6/T7 可传冗余轴目标角（A6→J7，T7→J2）。
    /// A6：给定 J7 时用 current_joints 作参考角做解析；未给定时从固定初值数值求解取出 J7，再解析。
    /// 解析公式用构造时的 DH。Set_DH_Para 改过 DH 后，锁住 J7 抛光到当前正运动学。
    /// T7 给定 J2 时走解析。T3、T6 走各自的解析解。其余走数值，初值是固定姿态。
    /// @param target 目标 TCP 齐次变换
    /// @param current_joints [in/out] 当前关节角（A6 给定 J7 时作参考角）；成功或部分失败时写入解
    /// @param target_redundant_joint 可选：冗余轴目标角 (rad)
    /// @return 0  成功，current_joints 已更新
    /// @return 1  解析解算出但关节超限，current_joints 已更新
    /// @return -1 目标不可达 / 无有效解 / 机型不支持
    /// @return -2 肘 / 腕 / 肩 / 几何奇异
    /// @return -3 解算结果关节超限
    /// @return -4 数值或抛光未收敛
    int Inverse_Kinematics(const Eigen::Matrix4d &target, Eigen::Ref<Eigen::RowVectorXd> current_joints,
                           std::optional<double> target_redundant_joint = std::nullopt);

    /// 关节空间五次多项式轨迹。
    /// @param start_joints 起始关节角
    /// @param target_joints 目标关节角
    /// @param joint_trajectory [out] 每行一帧关节角
    /// @return 0  成功，joint_trajectory 为完整路径
    /// @return -3 路径上存在关节超限，joint_trajectory 仍为完整路径
    int Joint_Trajectory(const Eigen::MatrixXd &start_joints, const Eigen::MatrixXd &target_joints,
                         Eigen::MatrixXd &joint_trajectory);

    /// 笛卡尔直线轨迹（TCP 直线，逐点 IK）。
    /// @param start_joints 起始关节角
    /// @param target [in/out] 目标 TCP 齐次变换；缩线 / 重规划时写回实际终点
    /// @param joint_trajectory [out] 每行一帧关节角
    /// @param target_redundant_joint 可选：冗余轴目标角 (rad)
    /// @return 0  成功，joint_trajectory 完整，target 为实际终点
    /// @return -1 无可行动 / 位姿序列失败，joint_trajectory 仅起点
    /// @return -2 路径经过奇异区，joint_trajectory 完整
    /// @return -3 中途触限，已从断点重规划，joint_trajectory 完整且末速归零
    /// @return -4 中途 IK 不可达或未收敛，已从断点重规划，joint_trajectory 完整且末速归零
    /// @return -5 中途 IK 与 FK 后验不一致，已从断点重规划，joint_trajectory 完整且末速归零
    /// @return -6 原目标不可达，已缩线至最远可行点，joint_trajectory 完整，target 已缩短
    /// @return -7 关节序列不连续，已从断点重规划，joint_trajectory 完整
    int Line_Trajectory(const Eigen::MatrixXd &start_joints, Eigen::Matrix4d &target,
                        Eigen::MatrixXd &joint_trajectory,
                        std::optional<double> target_redundant_joint = std::nullopt);

    /// 零重力与关节阻抗使用哪一套辨识参数。默认 0，即 xmin_current。
    /// 0 用 xmin_current，1 用 xmin_sensor。选中的 66 个数全为 0，或含非有限数时返回 false，不改当前选择。
    /// data_source 不是 0 或 1 时同样返回 false。碰撞检测始终用 xmin_sensor，不受这里影响。
    /// @param data_source 0 电流通道，1 传感器通道
    /// @return true 已切换到这一套
    bool set_Data_Source(int data_source = 0);

    /// 零重力：按关节写入 MIT 指令。非 A6 时 commands 清空。下标 0 为 J1。
    /// 调用方在循环外 reserve(7)。容量够时本函数只改写内容，不再分配。
    /// 每轴 q 为当前角，dq、kp、kd 为 0。tau 为牛·米，来自 set_Data_Source 选中的那一套。刚体项（含科氏）与零偏全额，摩擦乘 0.85。
    /// 速度先按 20 ms 一阶滤波。限位墙刚度 100、阻尼 10、缓冲 4°，进入本模式后 0.4 s 内从 0 加到满。
    /// 调用 Impedance_Step 后再进零重力，墙和滤波重新开始。
    /// 关节向量短于轴数时抛 std::invalid_argument。
    /// @param joint_positions 当前关节位置 (rad)
    /// @param joint_velocities 当前关节速度 (rad/s)，未滤波
    /// @param commands [out] 调用方持有的关节指令
    /// @param sample_period 控制周期 (s)。与 A6 控制环相同，用于滤波和限位墙渐加
    void Zero_Gravity_Step(const Eigen::Ref<const Eigen::RowVectorXd> &joint_positions,
                                   const Eigen::Ref<const Eigen::RowVectorXd> &joint_velocities,
                                   std::vector<Mit_Command> &commands,
                                   double sample_period);

    /// 关节阻抗：按关节写入 MIT 指令，不再返回电流。非 A6 时 commands 清空。下标 0 为 J1。
    /// 调用方在循环外 reserve(7)。容量够时本函数只改写内容，不再分配。
    /// 每轴 q 为期望角，dq 为 0。kp 为 80、80、50、50、30、30、30。
    /// kd = 0.4·√(kp·M)，M 为 1、1、0.5、0.5、0.2、0.2、0.2。
    /// tau 为牛·米，取当前角上的重力与零偏，来自 set_Data_Source 选中的那一套。位置弹簧在 kp、kd 里，交给电机。
    /// 调用本函数会结束零重力。下次再进零重力时，限位墙和速度滤波重新开始。
    /// current_joint_velocities 不使用。关节向量短于轴数时抛 std::invalid_argument。
    /// @param current_joints 当前关节位置 (rad)
    /// @param current_joint_velocities 保留签名，内部不使用
    /// @param desired_joints 期望关节位置 (rad)
    /// @param commands [out] 调用方持有的关节指令
    void Impedance_Step(const Eigen::Ref<const Eigen::RowVectorXd> &current_joints,
                                const Eigen::Ref<const Eigen::RowVectorXd> &current_joint_velocities,
                                const Eigen::Ref<const Eigen::RowVectorXd> &desired_joints,
                                std::vector<Mit_Command> &commands);

    /// 傅里叶辨识激励轨迹，返回 N×dof 关节角序列
    /// @param sample_period 采样周期 (s)
    Eigen::MatrixXd Fourier_Identification_Trajectory(double sample_period = 0.004) const;

    /// 由关节速度序列估计加速度 (rad/s²)。每行一拍，列数与轴数相同，顺序与采集一致。
    /// 速度先按 5 Hz 一阶低通，再一阶后向差分，与当前拍对齐。第 0 行加速度为 0。
    /// 行数小于 1、列数短于轴数、含非有限数，或 sample_period 非正、非有限时抛 std::invalid_argument。
    /// @param joint_velocities 采集到的关节速度 (rad/s)，未滤波
    /// @param sample_period 采集周期 (s)，与 Fourier_Identification_Trajectory 的周期相同
    Eigen::MatrixXd Joint_Acceleration(const Eigen::Ref<const Eigen::MatrixXd> &joint_velocities,
        double sample_period) const;

    /// 辨识动力学参数并热更新内存中的最小参数集。不写 yaml，由调用方决定是否落盘。
    /// data_source 选择力矩来源：0 电流换算力矩，写入 xmin_current；1 力矩传感器，写入 xmin_sensor。
    /// identify_coulomb、identify_viscous 为真时该项进入回归，为假时对应 7 个数写 0。默认都参与。
    /// 第 4 组已经是牛·米。回归前不再乘 Ktau。
    /// @param measurement_matrix 列布局：关节位置、速度、加速度、力矩(N·m)（各 dof）
    /// @param parameters [out] 辨识结果，66 个数：刚体 45、摩擦 14（每轴库伦、粘滞）、零偏 7。
    /// 求解成功即写入。返回 -1 / -2 / -3 时不改。
    /// @param data_source 0 电流力矩，1 传感器力矩
    /// @param identify_coulomb 是否辨识库伦摩擦
    /// @param identify_viscous 是否辨识粘滞摩擦
    /// @return 0  辨识完成，parameters 与内存参数已更新
    /// @return -1 非 A6 机型
    /// @return -2 输入矩阵为空、列数不足，或 data_source 不是 0/1
    /// @return -3 回归求解失败
    int Identify_Parameters(const Eigen::MatrixXd &measurement_matrix, Eigen::Ref<Eigen::RowVectorXd> parameters,
                            int data_source = 0, bool identify_coulomb = true, bool identify_viscous = true);

    /// 静态负载标定（A6）。同一姿态的带载与不带载力矩相减，7 个关节一起回归法兰系 [m, cx, cy, cz]（kg, m）。
    /// 采样姿态见当前手臂 yaml 的 payload_calib_path_deg（度，8 点，左右不同，不要改）。库不读取该键。
    /// empty、loaded 都是 8×14：前 7 列关节角 (rad)，后 7 列力矩 (N·m)。同一行是同一姿态。
    /// tool 全 0 时法兰系与工具系重合。成功后热更新内存中的 payload，不写 yaml。
    /// 关节角不一致不单独返回错误码。行数不是 8、列数不是 14，或含 NaN/Inf 时返回 -2。
    /// @param empty 不带载样本
    /// @param loaded 带载样本
    /// @param payload [out] 标定结果 [m, cx, cy, cz]
    /// @return 0  标定完成
    /// @return -1 非 A6 机型
    /// @return -2 不是 8×14，或含 NaN/Inf
    /// @return -3 姿态退化或条件数过大，解不可用
    int Calibrate_Payload(const Eigen::Ref<const Eigen::MatrixXd> &empty,
                          const Eigen::Ref<const Eigen::MatrixXd> &loaded, Eigen::Ref<Eigen::RowVector4d> payload);

    /// 写入末端负载 [m, cx, cy, cz]（kg, m），质心在工具系。只更新内存，不写 yaml。
    void Set_Payload(const Eigen::Ref<const Eigen::RowVector4d> &payload);

    /// 当前末端负载 [m, cx, cy, cz]（kg, m）。
    Eigen::RowVector4d Payload() const;

    /// 碰撞检测（A6，关节力矩传感器）。只判 J1–J3，触发后锁存。
    /// Clear_Collision 与 Set_Collision_Sensitivity 都会清除锁存。本函数不输出位置保持指令。
    /// 门限：档位×基值 + 库伦·|tanh(50·dq)| + 粘滞·|dq|，默认低档（×1）。
    /// @param joint_positions 当前关节位置 (rad)
    /// @param joint_velocities 当前关节速度 (rad/s)
    /// @param joint_torques_nm 关节力矩传感器读数 (N·m)，与 xmin_sensor 同通道
    /// @param sample_period 控制周期 (s)
    /// @return 0  本拍未超阈
    /// @return 2  已超阈并锁存（之后每拍都返回 2，直到清除）
    /// @return -1 非 A6 机型
    int Collision_Step(const Eigen::Ref<const Eigen::RowVectorXd> &joint_positions,
        const Eigen::Ref<const Eigen::RowVectorXd> &joint_velocities,
        const Eigen::Ref<const Eigen::RowVectorXd> &joint_torques_nm,
        double sample_period);

    /// 碰撞门限档位。只缩放基值。0 / 1 / 2 = 低 / 中 / 高 = ×1 / ×1.5 / ×2。默认 0。
    /// 改档位清除锁存和滤波，不复位动量观测器。与 A6 的 setCollisionThresholdTier 相同。
    void Set_Collision_Sensitivity(int sensitivity);

    /// 清除碰撞锁存，并复位传感器通道动量观测器。
    void Clear_Collision();

    /// 基座相对世界的旋转（基座→世界）。只改重力 G = Rᵀ(0,0,−9.81)，不改正逆解，不写 yaml。
    void Set_World_Base_Rotation(const Eigen::Matrix3d &R_world_base);

    /// 当前基座→世界旋转。构造时由 yaml 的 world_base（ZYX 欧拉角）写入；缺省为单位阵。
    Eigen::Matrix3d World_Base_Rotation() const;

private:
    class Robot;
    std::unique_ptr<Robot> robot_;
};

#endif // TI5_ROBOTICS_H
