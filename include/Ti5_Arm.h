#ifndef TI5_ARM_H
#define TI5_ARM_H

#include <Eigen/Dense>
#include <optional>
#include <string>
#include <vector>

/// 机械臂运动学解算器（T7/A6 七轴，T6 六轴）。
/// 对外关节角均为电机角 (rad)；模型角在实现内部转换。
class Robot_Arm
{
private:
    std::string model;
    std::string param_set;
    std::string side;
    std::string model_config_path;
    double sample_rate;
    double robot_dof;
    Eigen::Matrix<double, 1, 6> tool;
    Eigen::MatrixXd limit;
    Eigen::RowVectorXd model_to_motor_direct, model_to_motor_offset;
    Eigen::MatrixXd MDH;

public:
    std::vector<int> can_id_list;
    double Cart_Linear_Velocity = 0.10;

    /// @param model 机型，如 "T7"、"T6"、"A6"
    /// @param param_set 参数集，如 "T170"、"2kg"
    /// @param side 左/右臂，"left" 或 "right"
    /// @param model_config_path 模型参数 YAML 路径
    Robot_Arm(const std::string &model, const std::string &param_set, const std::string &side,
              const std::string &model_config_path = "../config/Robot_Arm_Model.yaml");

    /// @param current_joints_motor 当前关节电机角 (rad)，1×robot_dof（可传 Matrix<1,6> 或 Matrix<1,7>）
    Eigen::MatrixXd Jacobian(const Eigen::Ref<const Eigen::RowVectorXd> &current_joints_motor);

    /// @param current_joints_motor 当前关节电机角 (rad)，1×robot_dof（可传 Matrix<1,6> 或 Matrix<1,7>）
    /// @return 末端齐次变换矩阵 (4×4)
    Eigen::Matrix4d Forward_Kinematics(const Eigen::Ref<const Eigen::RowVectorXd> &current_joints_motor);

    /// 混合逆解：六轴走解析 T6；七轴先 Numeric 估计冗余关节（T7→J2，A6→J7），再解析精确求解。
    /// @param end_pose 目标位姿 [x,y,z,rx,ry,rz] (m, rad)
    /// @param current_joints_motor 输入电机角参考；输出电机角 (rad)，1×robot_dof
    /// @return 六轴：0 成功，-1 基座奇异，-2 肘部奇异，-3 腕部奇异，-4 关节超限
    ///         七轴：0 成功，-1 基座/未收敛，-2 肘部奇异，-3 腕部奇异或关节超限
    int Inverse_Kinematics(Eigen::Matrix<double, 1, 6> &end_pose, Eigen::Ref<Eigen::RowVectorXd> current_joints_motor);

    /// 解析逆解（六轴 T6；T7 约束 J2；A6 约束 J7）。
    /// @param end_pose 目标位姿 [x,y,z,rx,ry,rz] (m, rad)
    /// @param current_joints_motor 输入电机角参考；输出电机角 (rad)，1×robot_dof
    /// @param target_redundant_joint_motor T7 为 J2 电机角，A6 为 J7 电机角 (rad)；六轴忽略
    /// @return 六轴：0/-1/-2/-3/-4；七轴：0/-1/-2/-3（-3 含腕奇异与关节超限）
    int Inverse_Kinematics_Analytic(const Eigen::Matrix<double, 1, 6> &end_pose, Eigen::Ref<Eigen::RowVectorXd> current_joints_motor, double target_redundant_joint_motor);

    /// 数值逆解（阻尼最小二乘 + 零空间）。
    /// @param end_pose 目标位姿 [x,y,z,rx,ry,rz] (m, rad)
    /// @param current_joints_motor 输出电机角 (rad)，1×robot_dof
    /// @return 0 收敛成功；-1 达到最大迭代次数未收敛
    int Inverse_Kinematics_Numeric(Eigen::Matrix<double, 1, 6> &end_pose, Eigen::Ref<Eigen::RowVectorXd> current_joints_motor);

    /// 检查关节是否超出限位（内部 Motor_to_Model 后与 config 限位比较）。
    /// @param joints_motor 电机角 (rad)，1×robot_dof（可传 Matrix<1,6> 或 Matrix<1,7>）
    /// @return true 超出限位；false 在限位内
    bool Check_Joint_Limit(const Eigen::Ref<const Eigen::RowVectorXd> &joints_motor);

    /// 关节空间轨迹规划（五次多项式插值）。
    /// @param current_joints 起点电机角 (rad)，1×robot_dof
    /// @param joints_end 终点电机角 (rad)，1×robot_dof
    /// @param traj_out 输出轨迹电机角，(steps+1)×robot_dof；失败时为 current_joints
    /// @return 0 成功；-1 终点超限；-2 轨迹断点超限
    int Joint_Trajectory(const Eigen::MatrixXd &current_joints, const Eigen::MatrixXd &joints_end, Eigen::MatrixXd &traj_out);

    /// 笛卡尔直线轨迹规划。
    /// @param current_joints_motor 起点电机角 (rad)，1×robot_dof
    /// @param end_pose 目标位姿 [x,y,z,rx,ry,rz] (m, rad)
    /// @param traj_out 输出轨迹电机角，(steps+1)×robot_dof；失败时为 current_joints
    /// @param end_redundant_joint_motor 可选冗余关节电机角：T7→J2，A6→J7 (rad)；传入时沿路径插值冗余角
    /// @return 0 成功；-1 终点逆解失败；-2 轨迹断点逆解失败
    int Line_Trajectory(const Eigen::MatrixXd &current_joints_motor, Eigen::Matrix<double, 1, 6> &end_pose, Eigen::MatrixXd &traj_out,
                        std::optional<double> end_redundant_joint_motor = std::nullopt);
};

#endif
