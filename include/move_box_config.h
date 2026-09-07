#ifndef MOVE_BOX_CONFIG_H
#define MOVE_BOX_CONFIG_H

#include <Eigen/Dense>
#include <array>
#include <string>
#include <vector>

struct MoveBoxStandbyConfig
{
    Eigen::Matrix<double, 1, 6> right;
    Eigen::Matrix<double, 1, 6> left;
};

struct MoveBoxPlaceOffsetConfig
{
    double offset_x = 0.0;
    double offset_y = 0.0;
};

/** 某一排左右手放货 xy 补偿 */
struct MoveBoxPlaceRowHandOffsets
{
    MoveBoxPlaceOffsetConfig left;
    MoveBoxPlaceOffsetConfig right;
};

struct MoveBoxHeadGraspConfig
{
    double goal_z_base = -0.30;
    double goal_x_offset = 0.015;
    double goal_z_extra = 0.065;
    /** 头部分配后先到物体上方：z = 检测 z + hover_above_m（+z 朝上） */
    double hover_above_m = 0.08;
    double right_rx_deg = -90.0;
    double right_ry_deg = 45.0;
    double right_rz_deg = 0.0;
    double left_rx_deg = 90.0;
    double left_ry_deg = 45.0;
    double left_rz_deg = 0.0;
    double head_approach_z_descend = 0.05;
    double hand_descend_z = 0.08;
    double lift_after_grasp_z = 0.05;
    /** 手相机首次识别无效（无目标或数据超范围）后，最多重拍次数；0=不重拍 */
    int hand_detect_invalid_redo_max = 1;
    /** 手相机抓取 xy 偏移（视觉基座坐标上加） */
    MoveBoxPlaceOffsetConfig hand_right;
    MoveBoxPlaceOffsetConfig hand_left;
};

struct PlaceRowXBound
{
    double x_min = 0.0;
    double x_max = 0.0;
};

struct MoveBoxRow6BendConfig
{
    /** 与 row_enabled 第6项同时为真时，第6排走弯腰放货 */
    bool enabled = true;
    /** 0-based，5=第6排 */
    int row_index_0 = 5;
    /** pitch 补偿枢轴：base 下方距离(m)，现场 37.65cm */
    double pivot_z_below_base_m = 0.3765;
    double waist_pitch_deg = 20.0;
    double shoulder_lift_deg = 36.0;
    double right_rx_deg = -90.0;
    double right_ry_deg = 25.0;
    double right_rz_deg = 0.0;
    double left_rx_deg = 90.0;
    double left_ry_deg = 25.0;
    double left_rz_deg = 0.0;
    double speed_deg_per_s = 30.0;
    int smooth_dt_ms = 20;
    uint32_t waist_pitch_motor_can_id = 2;
    int right_shoulder_joint_index = 0;
    int left_shoulder_joint_index = 0;
    /** 弯放：先沿世界竖直抬高(m)，再 z_descend 下压；与 place.z_* 独立配置 */
    double z_raise = 0.06;
    double z_descend = -0.04;
    /** 放完一手后（腰仍弯）先沿世界竖直抬回 z_raise 高度，再移到下列 y/姿态 */
    double retreat_y_right = -0.15;
    double retreat_y_left = 0.15;
    /** 松爪后让位时 x 额外增加(m)，现场 10cm */
    double retreat_x_delta_m = 0.10;
    /** 让位与 x/y/姿态同动时，沿世界竖直再抬升(m) */
    double retreat_z_delta_m = 0.10;
    double retreat_right_rx_deg = -90.0;
    double retreat_right_ry_deg = -20.0;
    double retreat_right_rz_deg = 0.0;
    double retreat_left_rx_deg = 90.0;
    double retreat_left_ry_deg = -20.0;
    double retreat_left_rz_deg = 0.0;
};

struct MoveBoxPlaceConfig
{
    /** 第1~5排(0-based 0~4)放货 xy 补偿，每排左右手独立 */
    std::array<MoveBoxPlaceRowHandOffsets, 5> row_xy_offset{};
    /** 第6排(0-based 5)放货 xy 补偿，与 1~5 排独立配置 */
    MoveBoxPlaceRowHandOffsets row6_xy_offset;
    double z_raise = 0.0;
    /** 放货前相对已抬高位置再沿 z 移动（负=下降）；为 0 则跳过 */
    double z_descend = -0.02;
    /** 各排放货前腰 x 额外移动（place_advance 之后）；正=前进 负=后退；代码 pos(0)-=此值，手臂 x+=-此值。下标0=第1排 */
    std::vector<double> row_waist_x{-0.1, -0.1, -0.1, -0.1, -0.1, -0.1};
    /** 基座系 x 划分各排：[x_min, x_max)，下标0=第1排 */
    std::vector<PlaceRowXBound> row_x_bounds{
        {0.0, 0.4}, {0.4, 0.5}, {0.5, 0.6}, {0.6, 0.7}, {0.7, 0.8}, {0.8, 0.9}};
    /** 各排是否放货：1=启用 0=跳过；下标0=第1排 */
    std::vector<int> row_enabled{1, 1, 1, 1, 1, 1};
    /** 1=从第1排(x最小)往后排；0=从最后启用的排往前排 */
    int row_place_from_front = 1;
    /**
     * 48格棋盘放货顺序：1=先放44非基准(corr)，满后4基准(raw)；
     * 0=先放4基准(corr)，再放44非基准(corr)
     */
    int place_non_anchor_first = 1;
    /** 头相机空位检测次数（保留配置项；当前实现仅拍 1 次） */
    int detect_trials = 1;
    /** true：松爪前记录实际/下发 TCP 位姿到 picture_debug/place_pose.txt */
    bool place_pose_debug = false;
    /** 放货直线轨迹失败时改放此位（完整 x y z rx ry rz，与 standby 同格式 deg→rad） */
    Eigen::Matrix<double, 1, 6> fallback_right;
    Eigen::Matrix<double, 1, 6> fallback_left;
    /** 第6排弯腰放货（row_enabled 启用时生效） */
    MoveBoxRow6BendConfig row6_bend;
};

struct MoveBoxGraspValidConfig
{
    double z_min = -0.50;
    double z_max = -0.15;
    double x_min = 0.1;
    double x_max = 0.8;
    double right_y_min = -0.5;
    double right_y_max = 0.05;
    double left_y_min = -0.05;
    double left_y_max = 0.5;
};

struct MoveBoxWaistConfig
{
    Eigen::Matrix<double, 1, 6> layer3_home;
    /** 抓取腰进单步：正值=前进；代码 pos(0) -= 此值 */
    double stagger_step_x = 0.06;
    int stagger_max_steps = 2;
    /** 放货前腰进：正值=前进；代码 pos(0) -= 此值 */
    double place_advance_x = 0.12;
    int move_settle_sec = 2;
    /** 抓完放货前并行：底盘 +90° 启动前等待(秒) */
    int place_ready_chassis_delay_sec = 0;
    /** 抓完放货前并行：腰到放货预备位启动前等待(秒) */
    int place_ready_waist_delay_sec = 0;
    /** layer3 软件行程。x 减小=前进；z 增大=升高。x_min>=x_max 或 z_min>=z_max 则不限该轴 */
    double x_min = -0.30;
    double x_max = 0.10;
    double z_min = 0.50;
    double z_max = 0.72;
    /** 手臂系物体 z 低于此值则腰下降（躲开笛卡尔 z=-0.33 钳位）。0=关闭自动降腰 */
    double grasp_object_z_min = -0.28;
    /** 一次最多下降(m)，再被 z_min 截断 */
    double grasp_lower_z = 0.16;
};

struct MoveBoxStaggerConfig
{
    double head_x_threshold = 0.89;
};

/** 头相机抓取分配：侧区 / 中间区 y 分界（与 place_zone 独立） */
struct MoveBoxGraspZoneConfig
{
    /** 侧区 |y|>此值；中间区 [-y_side_split, y_side_split]，在 y=0 分左右半给双手 */
    double y_side_split = 0.05;
    /** 分配时丢弃 |y| 超过此值的目标（台面零件约 ≤0.20，椅子误检曾到 0.42） */
    double y_max_abs = 0.28;
    /** 分配时丢弃 x 小于此值的目标；x_min>=x_max 则不启用下限 */
    double x_min = 0.20;
    /** 分配时丢弃 x 超过此值的目标 */
    double x_max = 0.8;
    /**
     * 头相机画面边缘误检：投影点距图像边小于宽/高的此比例则丢。
     * 0=关闭。0.08 ≈ 1280×720 时左右 102px、上下 58px。
     */
    double edge_margin_frac = 0.08;
    /**
     * |cx|/cz、|cy|/cz 超过则当画面边缘（不依赖内参）。0=关闭。
     * 台面目标约 0.1~0.30，边角椅子误检约 0.58。
     */
    double cam_xy_over_z_max = 0.50;
};

struct MoveBoxPlaceZoneConfig
{
    /** 侧区分界：左手优先 y > y_side_split，右手优先 y < y_side_split */
    double y_side_split = 0.0;
    /** 中间区 y 上界（含），与 y_right 构成 [-0.08, 0.08] 一带 */
    double y_left = 0.08;
    /** 中间区 y 下界（含） */
    double y_right = -0.08;
};

struct MoveBoxArucoConfig
{
    int trials = 7;
    size_t grid_slot_count = 48;
};

/** 底盘充电电量阈值（percentage 0~1，与 getBatteryLevel 一致） */
struct MoveBoxBatteryConfig
{
    double low_threshold = 0.1;
    double full_threshold = 0.96;
};

/** RealSense 彩色/深度流（启动时 enable_stream） */
struct MoveBoxCameraConfig
{
    int width = 1280;
    int height = 720;
    /** 头相机帧率（原硬编码 30） */
    int head_fps = 15;
    /** 左右手相机帧率（原硬编码 30） */
    int hand_fps = 15;
};

/**
 * 传送带上方：以头相机看到的二维码为原点。
 * 码坐标系：+X=印刷右侧，+Y=印刷下方，+Z=垂直码面朝外（朝相机）。
 * 右侧上方 = 沿 +X 偏移 right_offset_m，再沿 +Z 抬高 height_above_m，右臂过去。
 * 左侧上方 = 沿 -X 偏移 left_offset_m，再沿 +Z 抬高，左臂过去。
 * 码正上方 = 无左右偏移，沿 +Z 抬高 above_height_m（默认 5cm）。
 */
struct MoveBoxConveyorConfig
{
    double right_offset_m = 0.15;
    double left_offset_m = 0.15;
    double height_above_m = 0.12;
    double above_height_m = 0.05;
    double speed = 0.2;
};

/** 笛卡尔直线用的逆解。T170 七轴冗余轴是 J2（电机角）。 */
enum class MoveBoxIkMethod
{
    Hybrid,   // Inverse_Kinematics：数值估 J2 再解析
    Analytic, // Inverse_Kinematics_Analytic：约束 J2
    Numeric,  // Inverse_Kinematics_Numeric：阻尼最小二乘
};

struct MoveBoxIkConfig
{
    MoveBoxIkMethod method = MoveBoxIkMethod::Analytic;
    /** true=锁当前 J2（暂不用于抓取直线）；false=不传冗余角 */
    bool j2_from_current = false;
    double j2_right_deg = 35.0;
    double j2_left_deg = -35.0;
};

/**
 * 各阶段视觉位姿算法（CirclePoseEngine algorithm_id）
 * -1 = 跟随 feeding_cylindrical_parts_alg/config/pose_params.yaml 各类别 algorithm_id
 *  0 = 强制 2D PnP（分割+椭圆，主要 RGB）
 *  1 = 强制 mask 深度重心（需对齐 depth，仅平移）
 */
struct MoveBoxVisionDetectConfig
{
    int head_grasp = 0;
    int right_hand_grasp = 0;
    int left_hand_grasp = 0;
    int place_holes = 0;
};

struct MoveBoxConfig
{
    MoveBoxStandbyConfig standby;
    MoveBoxHeadGraspConfig head_grasp;
    MoveBoxPlaceConfig place;
    MoveBoxGraspValidConfig grasp_valid;
    MoveBoxWaistConfig waist;
    MoveBoxStaggerConfig stagger;
    MoveBoxGraspZoneConfig grasp_zone;
    MoveBoxPlaceZoneConfig place_zone;
    MoveBoxArucoConfig aruco;
    MoveBoxBatteryConfig battery;
    MoveBoxCameraConfig cameras;
    MoveBoxVisionDetectConfig vision_detect;
    MoveBoxConveyorConfig conveyor;
    MoveBoxIkConfig ik;
};

MoveBoxConfig default_move_box_config();

std::string default_move_box_config_path();

bool load_move_box_config(const std::string &path, MoveBoxConfig &cfg, std::string &err);

void print_move_box_config(const MoveBoxConfig &cfg);

/**
 * 放货 xy 补偿：row_0 0~4=第1~5排，5=第6排；无效 row 回退第1排。
 * is_right=true 取右手，false 取左手。
 */
const MoveBoxPlaceOffsetConfig &place_hand_xy_offset(int row_0, bool is_right);

/** 由 main 加载；seg_pose_bridge 头部分配等读取 */
extern MoveBoxConfig g_move_cfg;

#endif
