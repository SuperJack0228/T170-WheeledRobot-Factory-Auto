#pragma once

#include <Eigen/Dense>
#include <string>

struct ArmPosePairConfig
{
    Eigen::Matrix<double, 1, 6> right = Eigen::Matrix<double, 1, 6>::Zero();
    Eigen::Matrix<double, 1, 6> left = Eigen::Matrix<double, 1, 6>::Zero();
};

struct HandXyOffsetConfig
{
    double offset_x = 0.0;
    double offset_y = 0.0;
};

/** 抓取末端姿态，yaml 单位 deg。 */
struct GraspRpyDeg
{
    double rx = 0.0;
    double ry = 60.0;
    double rz = 0.0;
};

struct TrayHeightConfig
{
    /** 基座系盘面 Z（卷尺/示教，向上为正）。对应量的时候那一档腰高。 */
    double z_ref_m = -0.72;
    /** 量 z_ref_m 时的腰 layer3.z（应与当时 home 一致）。腰升降时按差值改盘面参考。 */
    double z_ref_waist_z_m = 0.55;
    /** 毛坯相对盘面的典型露出高度。 */
    double part_above_tray_m = 0.015;
    /** 头/手相机绝对 Z 相对「参考顶面」超过该值则忽略相机高度。 */
    double z_refine_max_m = 0.01;
    /** 料盘位姿多帧融合帧数。1=单帧。 */
    int fuse_frames = 5;
};

struct HeadGraspConfig
{
    double hover_above_m = 0.08;
    double goal_z_base = -0.30;
    double goal_x_offset = 0.01;
    /** 左手示教：夹取点相对孔位（基座系）。右手用 right_goal_*，不取反。 */
    double goal_y_offset = 0.0;
    double goal_z_extra = 0.26;
    double right_goal_x_offset = 0.01;
    double right_goal_y_offset = 0.0;
    double right_grasp_z_offset_m = 0.02;
    double right_rx_deg = 0.0, right_ry_deg = 60.0, right_rz_deg = 45.0;
    double left_rx_deg = 0.0, left_ry_deg = 60.0, left_rz_deg = -45.0;
    /** 三组抓取 RPY，ready1/2/3。近远三排共用：0=row3/6，1=row2/5，2=row4。最远 row1 用 left_row6 存储的专用姿态。 */
    GraspRpyDeg left_row_rpy_deg[3];
    GraspRpyDeg right_row_rpy_deg[3];
    /** 最远一行（ArUco row1）专用末端姿态。yaml: row1_rpy_deg。 */
    GraspRpyDeg left_row6_rpy_deg;
    GraspRpyDeg right_row6_rpy_deg;
    /** true=二次 Bezier A-B-C；false=分段直线 A→B→C（B=物体上方，C=最终抓取）。Bezier 代码保留。 */
    bool use_bezier_grasp = false;
    double adaptive_pitch_down_deg = 45.0;
    double adaptive_yaw_max_deg = 55.0;
    double adaptive_min_horiz_m = 0.05;
    double adaptive_approach_xyz_tol_m = 0.03;
    double adaptive_approach_rpy_tol_deg = 12.0;
    double head_approach_z_descend = 0.05;
    double hand_descend_z = 0.07; // 已废弃，最终高度改用手相机 object_z
    /** 最终夹取 Z 修正，向上为正。偏深则加大，偏高则减小/改负。 */
    double grasp_z_offset_m = 0.02;
    /** 仅 from_robot=6（离机器人最近一排）额外 Z，叠在左右手 z_offset 上。向上为正。 */
    double nearest_row_grasp_z_offset_m = 0.0;
    /** false：跳过手相机，XY/Z 都用孔位+盘面参考。 */
    bool use_hand_camera = true;
    double hand_match_xy_max_m = 0.05;
    /** 手相机相对孔位 XY 的限幅微调；0 关闭。不覆盖 goal_x_offset。 */
    double hand_xy_refine_max_m = 0.03;
    HandXyOffsetConfig hand_right;
    HandXyOffsetConfig hand_left;
    double lift_after_grasp_z = 0.08;
    /** 二次 Bezier 控制点 B 位于最终点 C 正上方该距离。不把 B 抬过 A，而是先把 A 降下来拉开高度。 */
    double bezier_guide_height_m = 0.12;
    /** 规划前若 A.z 不低于 B.z−该值，先竖直降低 A（保持 XY/RPY）。 */
    double bezier_ab_gap_m = 0.06;
    /** Bezier 路径最大 TCP 速度；时间缩放会保证峰值不超过该值。 */
    double bezier_vel_m_s = 0.05;
    /** 前段保持待机姿态的路径比例；之后才转到抓取 RPY。 */
    double bezier_orient_finish_ratio = 0.65;
    /** 到 C 后 FK 实测位置/姿态超过阈值则禁止合爪。 */
    double bezier_endpoint_xyz_tol_m = 0.003;
    double bezier_endpoint_rpy_tol_deg = 3.0;
    /** 到上方 / 回起始点 的笛卡尔线速度（m/s）。 */
    double approach_vel_m_s = 0.12;
    /** 从上方下压的线速度，应明显慢于接近。 */
    double descend_vel_m_s = 0.05;
    double lift_vel_m_s = 0.10;
    double return_vel_m_s = 0.12;
    /** 下压到位后、合爪前等待（秒），避免还在晃就夹。 */
    double pre_grasp_settle_sec = 0.5;
    int hand_detect_invalid_redo_max = 10;
};

struct GraspValidConfig
{
    double z_min = -0.65, z_max = -0.05;
    double x_min = 0.10, x_max = 0.80;
    /** 工作包络。左右分列跟 ArUco 列号，不按盘心 Y + offset 拒目标。 */
    double right_y_min = -0.50, right_y_max = 0.28;
    double left_y_min = -0.28, left_y_max = 0.50;
};

struct GraspZoneConfig
{
    double y_side_split = 0.08; // 已废弃，列分区改用 column_split_y
    /** 已不再用于选孔；包络以 grasp_valid 为准。 */
    double y_max_abs = 0.50;
    double x_min = 0.10, x_max = 0.80;
    double edge_margin_frac = 0.08;
    double cam_xy_over_z_max = 0.50;
    /** 无料盘时 YOLO XY 回退用。料盘：同行最近、满排同时 (3-6/2-5/1-4)。 */
    double front_row_tolerance_m = 0.025;
    /** 仅无料盘位姿时的回退中缝（机器人 Y）。有 ArUco 盘心后改用 live 盘心 Y。 */
    double column_split_y = 0.0;
    double hole_pitch_m = 0.075;
    int column_count = 6;
    /** 同时抓最小列号差。3 表示 (1,4)(2,5)(3,6) 为最小对。 */
    int min_simultaneous_col_delta = 3;
    /** ArUco 行号 1=盘前方（远离机器人）。1..far_row_max 为远三排，抓前腰前伸。0=关闭。 */
    int far_row_max = 3;
};

struct WaistConfig
{
    Eigen::Matrix<double, 1, 6> layer3_home = Eigen::Matrix<double, 1, 6>::Zero();
    double stagger_step_x = 0.0556;
    int stagger_max_steps = 0;
    double move_settle_sec = 0.5;
    double x_min = -0.30, x_max = 0.30;
    double z_min = 0.35, z_max = 0.67;
    /** 抓取时盘面 live z_ref 目标。当前不再为够到该值而继续降腰。 */
    double grasp_object_z_min = -0.40;
    double grasp_lower_z = 0.16;
    /** ready/grasp 腰 TCP z（米）。home 仍用 layer3_home.z。 */
    double ready_z_m = 0.36;
    /** ready/grasp 起始：相对 layer3_home.x 再向前（米，+X 为机器人前方）。home 仍停在 layer3_home.x。 */
    double ready_forward_m = 0.06;
    /** 远三排 / waist2：相对 layer3_home.x 再向前伸的距离（米）。抓完不收回。 */
    double far_row_forward_m = 0.15;
};

/** 传送带工位 XYZ 偏置（米，基座系）。叠在 tcp 上，右手不取反。 */
struct ConveyorXyzOffset
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/** 仙工底盘握手。抓好后导航到 belt_station，等到站再放置。 */
struct ChassisConfig
{
    std::string host = "192.168.192.5";
    std::string tray_station = "AP4";
    /** 第二个料盘（空孔放置）。调试指令 tray2 / tray2ready*。 */
    std::string tray2_station = "AP9";
    std::string belt_station = "AP6";
    /** 第二条传送带。cycle 第一次抓完腰回 0 后去这里再放置。 */
    std::string out_station = "AP5";
    /** 等站点到位超时（毫秒）。abort 可提前打断。 */
    int nav_timeout_ms = 600000;
};

/** 料盘2空孔放置的四组 TCP 姿态。与 head_grasp ready1/2/3/6 独立，方便单独微调。 */
struct Tray2PlaceConfig
{
    GraspRpyDeg left_row_rpy_deg[3];
    GraspRpyDeg right_row_rpy_deg[3];
    GraspRpyDeg left_row6_rpy_deg;
    GraspRpyDeg right_row6_rpy_deg;
    /** 相对孔位的放置偏置（基座系）。左右分开，不共用 head_grasp。 */
    double goal_x_offset = 0.015;
    double goal_y_offset = -0.01;
    double grasp_z_offset_m = 0.02;
    double right_goal_x_offset = 0.01;
    double right_goal_y_offset = -0.01;
    double right_grasp_z_offset_m = 0.04;
    /** 仅 from_robot=6，叠在该手 z 上。 */
    double nearest_row_z_offset_m = -0.01;
};

/** 传送带工位调试姿态。与料盘 waist/head/standby/home_tcp 独立。
 *  `belt_ready`：腰/头 + tcp（准备）。`belt_place`：腰/头 + place_tcp（放置末端）。
 *  `belt`：准备后认码，先右手放到码+offset再回准备，再左手；不回 home。 */
struct ConveyorStationConfig
{
    double waist_x = -0.15;
    double waist_z = 0.63;
    double head_yaw_deg = 0.0;
    double head_pitch_deg = 50.0;
    double head_roll_deg = 0.0;
    ArmPosePairConfig tcp;
    /** 放置末端姿态。belt_place 预览整段 6D；完整 belt 只用其 RPY。 */
    ArmPosePairConfig place_tcp;
    /** 相对识别板系（+X 前、+Y 左、+Z 上）：放置 XYZ = 码原点 + R_识别 × offset。 */
    ConveyorXyzOffset offset_right;
    ConveyorXyzOffset offset_left;
    /** 优先用哪条皮带（belt0/belt1）。该条没码则用第一条解到的。 */
    std::string prefer_belt = "belt0";
    /** 传送带抓取专用 RPY（deg）。XYZ 用左右各自的 grasp_offset。 */
    GraspRpyDeg grasp_rpy_left{0.0, 45.0, -30.0};
    GraspRpyDeg grasp_rpy_right{0.0, 45.0, 30.0};
    /** 相对识别板系（+X 前、+Y 左、+Z 上）。不转腰，不再乘腰 Rz。 */
    ConveyorXyzOffset grasp_offset_left;
    ConveyorXyzOffset grasp_offset_right;
    /** 抓取完成后回到这里。开始准备仍用 tcp。未配置则等于 tcp。 */
    ArmPosePairConfig grasp_tcp;
    /** 抓取点上方接近高度。0=直接落到抓取点。 */
    double grasp_hover_above_m = 0.06;
    /** 放置点上方接近高度。先到该高度再下降松爪。0=直接落到放置点。 */
    double place_hover_above_m = 0.06;
    /** 叠在放置目标的基座 Z 上（米，向上为正）。0=不改。转运松手再降用负值。 */
    double place_base_z_bias_m = 0.0;
    /** 传送带抓取转腰（deg）。0=不转。站点更新后默认不转。 */
    double grasp_waist_yaw_deg = 0.0;
    /** 准备姿态夹爪闭合比例，0=全开 1=全合。减小开距，避免下探时磕皮带边。 */
    double grasp_ready_close_ratio = 0.30;
    /** 抓取前是否跑 YOLO 类别门禁。false=不跑 YOLO，只认码。 */
    bool grasp_yolo_enable = false;
    /** 抓取前 YOLO 必须看到的类别。仅判定，不参与定位。1=半加工料 2=精加工料；<0 不检查。 */
    int grasp_yolo_class_id = -1;
    /** true：码原点 Z 不用相机，改用 fixed_origin_z_m。XY 和板姿态仍用二维码。 */
    bool use_fixed_origin_z = false;
    /** 手臂基座系码平面高度（米，向上为正）。腰 0.63m 时传送带在基座下方 0.29m。 */
    double fixed_origin_z_m = -0.29;
};

struct StaggerConfig
{
    double head_x_threshold = 0.80;
};

struct VisionDetectConfig
{
    int head_grasp = 1;
    int right_hand_grasp = 1;
    int left_hand_grasp = 1;
};

struct CameraStreamConfig
{
    int width = 1280;
    int height = 720;
    int head_fps = 15;
    int hand_fps = 15;
};

/** 头相机标定关节角。CAN 30=偏航 32=俯仰 31=横滚。 */
struct HeadMotorConfig
{
    double yaw_deg = 0.0;
    double pitch_deg = 40.0;
    /** 腰前伸抓远三排时的低头俯仰。 */
    double far_pitch_deg = 60.0;
    double roll_deg = 0.0;
    double speed_deg_s = 25.0;
};

struct MoveBoxConfig
{
    ArmPosePairConfig standby;
    ArmPosePairConfig home_tcp;
    TrayHeightConfig tray;
    HeadMotorConfig head;
    HeadGraspConfig head_grasp;
    GraspValidConfig grasp_valid;
    GraspZoneConfig grasp_zone;
    WaistConfig waist;
    ConveyorStationConfig conveyor;
    /** 第二条传送带。缺省抄 conveyor；yaml conveyor2 可单独改。 */
    ConveyorStationConfig conveyor2;
    Tray2PlaceConfig tray2_place;
    ChassisConfig chassis;
    StaggerConfig stagger;
    VisionDetectConfig vision_detect;
    CameraStreamConfig cameras;
};

extern MoveBoxConfig g_move_cfg;

MoveBoxConfig default_move_box_config();
std::string default_move_box_config_path();
bool load_move_box_config(const std::string &path, MoveBoxConfig &cfg, std::string &err);
void print_move_box_config(const MoveBoxConfig &cfg);

double column_split_y();
/** 本轮料盘原点 Y（基座系）。有 ArUco 解算时覆盖 yaml column_split_y。 */
void set_live_tray_origin_y(double origin_y);
void clear_live_tray_origin_y();
bool have_live_tray_origin_y();
int tray_column_index_from_y(double y);
/** ArUco 列 → 左右手列号。盘转 180° 时 flipped=true，列号对调。1–3 右、4–6 左。 */
int tray_assign_col_from_aruco(int aruco_col, bool cols_flipped);
bool tray_assign_col_is_right(int assign_col);
/** ArUco 行 1 起为盘前方（远处）。 */
bool tray_row_is_far(int row);
/** 行姿态组：ready1=row3/6，ready2=row2/5，ready3=row4；row1（最远）返回 1 专用。 */
int tray_row_pose_group(int row);
const GraspRpyDeg &grasp_rpy_deg_for_row(int row, bool is_right);
/** ready 指令 1/2/3/6 对应的抓取 RPY。 */
const GraspRpyDeg &grasp_rpy_deg_for_ready(int ready_id, bool is_right);
/** 料盘2放置专用 RPY，不读 head_grasp。 */
const GraspRpyDeg &tray2_place_rpy_deg_for_row(int row, bool is_right);
const GraspRpyDeg &tray2_place_rpy_deg_for_ready(int ready_id, bool is_right);
const char *ready_rows_label(int ready_id);
/** 孔位 XYZ 偏置：左右手各用 yaml 自己的一套（基座系，右手不取反）。 */
double grasp_goal_x_offset(bool is_right);
double grasp_goal_y_offset(bool is_right);
double grasp_goal_z_offset(bool is_right, int from_robot_row = 0);
bool arm_y_allowed_right(double y);
bool arm_y_allowed_left(double y);
bool simultaneous_columns_ok(double y_right, double y_left);
bool simultaneous_assign_columns_ok(int col_right, int col_left);
void log_arm_wall_reject(const char *stage, bool is_right, double y);
/** ready/grasp 起始腰 X = layer3_home.x + ready_forward_m。 */
double waist_ready_x();
/** ready/grasp 腰 Z，独立于 home。 */
double waist_ready_z();
/** 远三排 / waist2 腰 X = layer3_home.x + far_row_forward_m。 */
double waist_far_row_x();
/** 腰向左为正。T_arm←chassis = Rz(-θ)，把转腰前示教的 XY 变到当前手臂基座。 */
void conveyor_chassis_xy_to_arm_base(double yaw_rad, double &x, double &y);
