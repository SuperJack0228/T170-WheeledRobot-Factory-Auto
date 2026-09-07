#ifndef FUNCTION_H
#define FUNCTION_H

#include "head.h"
#include "Ti5_Arm.h"

using namespace Eigen;
using namespace std;



//class function
//{
//public:
//    function() {

//    }


//};
//#define pi acos(-1)
//double rad=pi/180,deg=180/pi;
// double rad(),deg();

MatrixXd Transl_xyz(Matrix<double, 1, 3> trans);
MatrixXd Rot_zyx(Matrix<double, 1, 3>  theta);
MatrixXd TR(Matrix<double, 1, 6> transpose);
MatrixXd MDHTrans(double alpha, double  a, double  d, double  theta);
Matrix<double, 1, 3> rotationMatrixToEulerAngles(Matrix<double, 3, 3> R);
Matrix<double, 1, 6> T2PosEulerAngles(Matrix<double, 4, 4> T);
Eigen::Vector3d Quaterniond2EulerAngles(Eigen::Quaterniond q);
//MatrixXd T_to_AngleAxis(Matrix<double, 4, 4>  T);
MatrixXd T2Axispos(Matrix<double, 4, 4> T);
Eigen::VectorXd T2Axis(const Eigen::Matrix4d& T) ;
MatrixXd Axispos2T(Matrix<double, 1, 6> pos);
// MatrixXd ndi2T(Matrix<double, 1, 6>  pos);
// MatrixXd displaypos(Matrix<double, 1, 6>  pos);
MatrixXd skew(Matrix<double, 1, 3>  s);
//正运动学 根据输入关节角度求出末端法兰相对于基座4*4矩阵
MatrixXd fkine(Matrix<double, 1, 6>  theta);

// Matrix<double, 4, 4>  eyehand_falan_calib(Matrix<double, 8, 6> Joint, Matrix<double, 8, 6> NDIpose);



int getFileRows(const char* fileName);
int getFileColumns(const char* fileName);
double** getMatrix(const char* path, const int n, const int m);


MatrixXd line_interp(Matrix<double, 1, 6>   p1,Matrix<double, 1, 6>  p2, double dx);
MatrixXd circular_interp(Matrix<double, 1, 6>   p1,Matrix<double, 1, 6>  p2,Matrix<double, 1, 6>  p3, double dx);
MatrixXd pinv(Eigen::MatrixXd A);//计算矩阵伪逆
MatrixXd multiplyWithDOF(Matrix<double,1 , 6> dof,Matrix<double,1 , 6> forces);
MatrixXd slerp_interp(double t, Matrix<double, 1, 3> euler_start, Matrix<double, 1, 3> euler_end);


Matrix<double, 1, 6> quinticInterp_CartSpace(double t, double totalTime, const Eigen::Matrix<double, 1, 6>& start,  const Eigen::Matrix<double, 1, 6>& target);
Matrix<double, 1, 7> quinticInterp_JointSpace(double t, double totalTime, const Eigen::Matrix<double, 1, 7>& start,  const Eigen::Matrix<double, 1, 7>& target);
std::array<double, 3> extractCoordinates(const std::string& input);



Eigen::Matrix3d Yaw_Rotation(float yaw);


Eigen::Matrix3d Pitch_Rotation(float pitch);


Eigen::Matrix3d Roll_Rotation(float roll);


Eigen::Matrix4d neck_to_eye(int flag);

double calculateTf(const Eigen::Matrix<double, 1, 6>& axis_p1,
                   const Eigen::Matrix<double, 1, 6>& axis_p2,
                   double V_DEFAULT);
Matrix<double, 1, 7> joint_range_normalize(const Eigen::Matrix<double, 1, 7>& q,const Eigen::Matrix<double, 1, 7>& q_min,const Eigen::Matrix<double, 1, 7>& q_max);

Matrix<double, 1, 7> calc_segment_weight( const Eigen::Matrix<double, 1, 7>& x,  double w_max) ;

std::tuple<MatrixXd, MatrixXd, MatrixXd>
quinticInterp(
    double t,
    double totalTime,
    const MatrixXd& start,  // 统一 Matrixd：1×N
    const MatrixXd& target  // 统一 Matrixd：1×N
);

/** z 低于 grasp_valid.z_min 时警告并钳到 z_min；where 为产生/下发该 goal 的位置说明 */
void clamp_cart_goal_z(Eigen::Matrix<double, 1, 6> &pos, const char *where);

/** Line_Trajectory 返回值：0=成功，非0=轨迹解析失败 */
struct ArmLineMoveResult
{
    int ret_r = 0;
    int ret_l = 0;
};

/** 设置当前直线段阶段名（供 Line_Trajectory 失败时写入 picture_debug/line_trajectory_fail.txt） */
void arm_line_move_set_debug_stage(const char *stage);

/** RAII：在作用域内标注 arm_line_move 阶段名 */
class ArmLineMoveDebugStage
{
public:
    explicit ArmLineMoveDebugStage(const char *stage) { arm_line_move_set_debug_stage(stage); }
    ~ArmLineMoveDebugStage() { arm_line_move_set_debug_stage(nullptr); }
    ArmLineMoveDebugStage(const ArmLineMoveDebugStage &) = delete;
    ArmLineMoveDebugStage &operator=(const ArmLineMoveDebugStage &) = delete;
};

/** Line_Trajectory 非 0 时追加写入 picture_debug/line_trajectory_fail.txt */
void append_line_trajectory_fail_debug(
    const char *hand_label,
    int ret,
    const Eigen::Matrix<double, 1, 6> &goal,
    const char *stage,
    double cart_linear_velocity);

/** 读编码器 → Line_Trajectory → 每 5ms set_motor_position。
 * 不锁 J2（不传冗余角），让七轴逆解自己选肘，保证抓取工作空间。 */
int arm_line_move(Robot_Arm &arm, Matrix<double, 1, 6> &pos, double cart_linear_velocity);

/** 与 arm_line_move 相同（笛卡尔直线到目标）。 */
int arm_transfer_move(Robot_Arm &arm, Matrix<double, 1, 6> &pos, double cart_linear_velocity);

ArmLineMoveResult arm_dual_transfer_move(
    Robot_Arm &arm_r,
    Matrix<double, 1, 6> &pos_r,
    Robot_Arm &arm_l,
    Matrix<double, 1, 6> &pos_l,
    double cart_linear_velocity);

ArmLineMoveResult arm_dual_transfer_move_selective(
    Robot_Arm &arm_r,
    Matrix<double, 1, 6> &pos_r,
    bool move_r,
    Robot_Arm &arm_l,
    Matrix<double, 1, 6> &pos_l,
    bool move_l,
    double cart_linear_velocity);

/**
 * 连杆原点（基座系，米）。SDK Forward_Kinematics 只给 TCP；
 * 肘/腕用 config/Robot_Arm_Model.yaml 的 T170 MDH 累乘（与 SDK TCP 对齐后的那套电机→模型换算）。
 * - shoulder: J2 原点
 * - elbow:    J4 原点（上臂末端）
 * - wrist:    J5 原点（腕中心）
 * - flange:   J7 原点（法兰，未加工具 230mm）
 * - tcp:      SDK 末端
 */
struct ArmLinkFk
{
    Eigen::Vector3d shoulder = Eigen::Vector3d::Zero();
    Eigen::Vector3d elbow = Eigen::Vector3d::Zero();
    Eigen::Vector3d wrist = Eigen::Vector3d::Zero();
    Eigen::Vector3d flange = Eigen::Vector3d::Zero();
    Eigen::Vector3d tcp = Eigen::Vector3d::Zero();
    double tcp_mdh_err = 0.0;
};

ArmLinkFk arm_fk_links(Robot_Arm &arm, const Eigen::Ref<const Eigen::RowVectorXd> &q_motor);
/** 读当前编码器再算连杆 FK */
ArmLinkFk arm_get_link_fk(Robot_Arm &arm);
void log_arm_link_fk(Robot_Arm &arm, const char *tag);

/** 左右臂双线程同时 line_move */
ArmLineMoveResult arm_dual_line_move(
    Robot_Arm &arm_r,
    Matrix<double, 1, 6> &pos_r,
    Robot_Arm &arm_l,
    Matrix<double, 1, 6> &pos_l,
    double cart_linear_velocity);

/** move_r/move_l 为 false 的一侧不移动；可只动单臂或双臂 */
ArmLineMoveResult arm_dual_line_move_selective(
    Robot_Arm &arm_r,
    Matrix<double, 1, 6> &pos_r,
    bool move_r,
    Robot_Arm &arm_l,
    Matrix<double, 1, 6> &pos_l,
    bool move_l,
    double cart_linear_velocity);

/** 记录最近一次 set_motor_position 下发（供 wait_arms_motors_stopped 编码器校验） */
void save_arm_last_commanded_position(int hand, const int motor_cmd[7]);

/** hand: 1=右手 0=左手；7 轴中至少 6 轴 speed==0 视为停稳 */
bool arm_motors_all_stopped(int hand, int speed_threshold = 0);

/** 轮询直到指定侧停稳或超时：速度停稳暂关闭，仅编码器；
 *  ±550 直接通过；>±550 且连续10次变化≤10 视为稳定到位 */
bool wait_arms_motors_stopped(
    bool wait_r,
    bool wait_l,
    double timeout_sec = 15.0,
    int min_zero_count = 6,
    const char *log_stage = "手相机前");

/** 读 CAN 命令6 并打印左右手 7 轴速度（手相机停稳确认用） */
void log_arms_motor_running_speeds(bool log_r, bool log_l, const char *reason);

/** 终端分隔：各流程阶段（=×80） */
constexpr const char kLogPhaseSep[] =
    "================================================================================";
/** 终端分隔：主循环每轮开始（*×80） */
constexpr const char kLogCycleSep[] =
    "********************************************************************************";

void log_phase_banner(const char *title);
/** round_index>=0 时打印轮次；<0 仅打印新一轮 */
void log_cycle_banner(int round_index = -1);

int arm_joint_move(Robot_Arm &arm, Matrix<double, 1, 7> &qd);
/** 读编码器 → 关节角(rad) → FK 4x4 → 1x6(x,y,z m; rx,ry,rz 欧拉角 rad) */
Matrix<double, 1, 6> arm_get_tcp_pos(Robot_Arm &arm);
/** 读编码器→rad→FK→法兰×视觉位姿→基座 1×6（读数路径同 arm_get_tcp_pos） */
Matrix<double, 1, 6> arm_get_base_visual_pos(Robot_Arm &arm, Matrix<double, 1, 6> &falan_visualPos);

#endif // FUNCTION_H
