/**
 * @file gripper_interface.hpp
 * @brief DEX1 夹爪直接串口 C++ 接口（无 ROS2 / 无 DDS）
 *
 * 协议与宇树 dex1_1_service 一致（M4010 电机）：
 *   https://github.com/unitreerobotics/dex1_1_service
 *
 * ## 坐标与 ID
 * - 关节角 q：open_position_rad(5.5)=全开，close_position_rad(0)=全合
 * - motor_id 0=右夹爪，1=左夹爪（宇树固定，与 USB 口无关）
 * - 启动时扫描 /dev/ttyUSB*、/dev/ttyCH343USB* 自动绑定左右
 *
 * ## 两套 API（互不删除，可并存）
 *
 * ### 1. 经典接口（默认，搬箱推荐）
 * - open/close/graspAndWait：全速（default_slew_rate=0），力矩保护靠监测 |torque|
 * - setTeleopRatio：遥操简单力矩锁（|torque|≥limit 时锁在当前 q）
 *
 * ### 2. 软力矩接口 Soft*（参考 ros2_170D dex1_gripper_driver_node）
 * - openSoft/closeSoft/graspSoftAndWait：斜率限制 + MIT 阻抗 kp/kd 反算钳位 q_cmd
 * - setTeleopSoftRatio：遥操 + 同款软限位
 * - 参数：soft_torque_limit_nm、pos_filter、soft_slew_rate
 *
 * 参考：ros2_170D/src/gripper_driver/src/dex1_gripper_driver_node.cpp
 */
#ifndef GRIPPER_GRIPPER_INTERFACE_HPP
#define GRIPPER_GRIPPER_INTERFACE_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace gripper {

/** @brief 夹爪侧别（逻辑左右，与 motor_id 映射见文件头说明） */
enum class Side { Left, Right };

/**
 * @brief 全局配置
 *
 * 经典搬箱用 default_slew_rate / grasp_torque_limit_nm；
 * Soft* 接口用 soft_slew_rate / soft_torque_limit_nm / pos_filter。
 */
struct Config {
  /** @brief 对外角度 0~full_close_angle_deg 映射到 q 的比例尺（默认 1.0°） */
  double full_close_angle_deg = 1.0;
  /** @brief 全开关节角 (rad)，对应 DEX q≈5.5 */
  double open_position_rad = 5.5;
  /** @brief 全合关节角 (rad)，对应 DEX q≈0 */
  double close_position_rad = 0.0;
  /** @brief MIT 位置刚度 kp（经典与 Soft 共用） */
  double kp = 5.0;
  /** @brief MIT 速度阻尼 kd（经典与 Soft 共用） */
  double kd = 0.05;
  /**
   * @brief 经典接口斜率 (rad/s)。
   * 0=立即跳到目标（open/close/grasp 默认）；>0 则 q_cmd 平滑逼近 target_q。
   */
  double default_slew_rate = 0.0;
  /** @brief 斜率上限（保留，与 dex1_1_service 对齐） */
  double max_slew_rate = 50.0;
  /** @brief 最大闭合比例 0~1，1=全合 */
  double max_close_ratio = 1.0;
  /** @brief 控制线程频率 (Hz)，默认 200 */
  double control_hz = 200.0;
  /** @brief 扫描串口 probe 重试次数 */
  int detect_retries = 2;
  /** @brief 标定闭合限位角 (rad)，默认 322° */
  float calibration_limit_rad = 322.0f * 3.14159265358979323846f / 180.0f;

  // --- 经典力矩限位（graspAndWait / setTeleopRatio）---
  /**
   * @brief 经典夹取/遥操力矩上限 (N·m，关节侧)。
   * graspAndWait 中 |torque|≥此值即停；遥操往闭合方向时触顶锁 q。0=关闭。
   */
  double grasp_torque_limit_nm = 1.0;
  /** @brief 经典遥操斜率 (rad/s)，建议 4~10 */
  double teleop_slew_rate = 10.0;

  // --- ros2_170D 软力矩限位（Soft* / setTeleopSoftRatio）---
  /**
   * @brief 软力矩上限 (N·m)，对应 ros2 soft_torque_limit。
   * MIT 反算：q_lo/hi = q_anchor + (±limit + kd*dq_filt)/kp。0=关闭软限位钳位。
   */
  double soft_torque_limit_nm = 4.0;
  /** @brief q/dq 低通滤波系数 (0~1)，对应 ros2 pos_filter，默认 0.2 */
  double pos_filter = 0.35;
  /**
   * @brief Soft* 接口斜率 (rad/s)，仅用于接近目标末段；大行程直接跟目标。
   */
  double soft_slew_rate = 100.0;
  /**
   * @brief 距目标超过此值 (rad) 时 q_cmd 直接等于 target（与经典同速）。
   * 进入该范围内才斜率+软力矩缓停。
   */
  double soft_coast_margin_rad = 0.12;
};

/** @brief connect() 后单电机探测结果 */
struct DetectedMotor {
  Side side = Side::Left;
  int motor_id = 0;
  std::string port;
};

/** @brief graspAndWait / graspSoftAndWait 结束原因 */
enum class GraspResult {
  TorqueLimit,      ///< 碰到物体，力矩到上限后停住
  PositionReached,  ///< 空夹，闭合到位
  Timeout,          ///< 超时仍未满足上述条件
};

/** @brief 夹取阻塞接口的返回值 */
struct GraspFeedback {
  GraspResult result = GraspResult::Timeout;
  double position_rad = 0.0;  ///< 结束时关节角 q (rad)
  double torque = 0.0;        ///< 结束时力矩 (N·m)
};

/** @brief feedback() 单侧状态（经典 + Soft 共用） */
struct SideFeedback {
  double angle_deg = 0.0;         ///< 映射角度 (°)，0≈开，full_close≈合
  double position_rad = 0.0;      ///< 关节角 q (rad)
  double velocity_rad_s = 0.0;    ///< 关节速度 dq (rad/s)
  double torque = 0.0;            ///< 估计力矩 tau (N·m)
  bool have_feedback = false;     ///< 是否已成功读过至少一帧 state
  double command_rad = 0.0;       ///< 控制线程实际下发的 q_cmd (rad)
  /**
   * @brief Soft 模式下本周期是否被 kp/kd 反算钳位。
   * 经典模式下恒为 false（经典遥操请看 TeleopFeedback::torque_limited）。
   */
  bool soft_torque_limited = false;
  /** @brief 最近一帧 MError；bit9=512 为 HG 超时保护 */
  std::uint32_t merror = 0;
  /** @brief 最近一次 set_Gripper_Pos_get_State 是否收到回包 */
  bool io_ok = false;
};

/**
 * @brief 遥操轮询反馈（setTeleopRatio / setTeleopSoftRatio）
 */
struct TeleopFeedback {
  double command_ratio = 0.0;    ///< 手柄下发 0~1（0=开，1=合）
  double effective_ratio = 0.0;  ///< 限位后的实际目标比例（映射到 q）
  double position_rad = 0.0;
  double torque = 0.0;
  /** @brief 是否触顶限位（经典=|tau|锁 q；Soft=q_cmd 被钳位） */
  bool torque_limited = false;
};

/**
 * @brief 扫描系统中可用的夹爪串口设备路径
 * @return /dev/ttyUSB* 与 /dev/ttyCH343USB* 列表（已排序）
 */
std::vector<std::string> scanSerialPorts();

/**
 * @class Gripper
 * @brief DEX1 双夹爪控制器（直接 USB 串口，无需 dex1_1_gripper_server）
 *
 * 典型用法：
 * @code
 * gripper::Gripper g;
 * g.start();
 * g.openAndWait(gripper::Side::Left);           // 经典
 * g.graspAndWait(gripper::Side::Left);
 * g.openSoftAndWait(gripper::Side::Right);      // ros2 软限位
 * g.closeSoftAndWait(gripper::Side::Right);
 * g.stop();
 * @endcode
 */
class Gripper {
public:
  /** @brief 使用 config 构造（可拷贝配置后传入自定义 Config） */
  explicit Gripper(const Config &config = Config{});
  ~Gripper();

  Gripper(const Gripper &) = delete;
  Gripper &operator=(const Gripper &) = delete;

  // =========================================================================
  // 生命周期
  // =========================================================================

  /**
   * @brief 扫描串口并 probe motor_id 0/1，绑定左右（不启动控制线程）
   * @return 至少检测到一侧电机则 true
   */
  bool connect();

  /** @brief connect() + 启动 200Hz 控制线程 */
  bool start();

  /** @brief 停止控制线程并断开，释放串口 */
  void stop();

  /** @brief 是否已成功 connect() */
  bool isConnected() const;

  /** @brief 控制线程是否在运行 */
  bool isRunning() const { return running_.load(); }

  /** @brief 返回已识别电机列表（side / motor_id / port） */
  std::vector<DetectedMotor> detectedMotors() const;

  /** @brief 是否检测到该侧电机 */
  bool hasSide(Side side) const;

  /** @brief 只读配置 */
  const Config &config() const { return config_; }

  // =========================================================================
  // 经典接口 — 非阻塞目标设置（搬箱全速，default_slew_rate 默认 0）
  // =========================================================================

  /**
   * @brief 按角度 (°) 设目标，0=开，full_close_angle_deg=合
   * @note 会退出遥操模式；会关闭该侧 Soft 模式
   */
  void setAngle(Side side, double angle_deg);

  /** @brief 双手同时 setAngle */
  void setBothAngles(double left_deg, double right_deg);

  /** @brief 经典张开（target_q=open_position_rad，立即 q_cmd） */
  void open(Side side);

  /** @brief 经典双手张开 */
  void openBoth();

  /**
   * @brief 经典闭合
   * @param ratio 闭合比例 0~1，1=全合到 close_position_rad
   */
  void close(Side side, double ratio = 1.0);

  /** @brief 经典双手闭合 */
  void closeBoth(double ratio = 1.0);

  // =========================================================================
  // 经典接口 — 阻塞等待
  // =========================================================================

  /**
   * @brief 经典张开并阻塞等到 |q - open| ≤ tolerance
   * @return 到位 true；超时 false
   */
  bool openAndWait(Side side, double tolerance_rad = 0.2,
                   std::chrono::milliseconds timeout = std::chrono::seconds(3));

  /** @brief 经典双手张开并等待 */
  bool openBothAndWait(double tolerance_rad = 0.2,
                       std::chrono::milliseconds timeout = std::chrono::seconds(3));

  /**
   * @brief 经典闭合并阻塞等待
   * @param ratio 闭合比例 0~1
   */
  bool closeAndWait(Side side, double ratio = 1.0, double tolerance_rad = 0.2,
                    std::chrono::milliseconds timeout = std::chrono::seconds(3));

  /** @brief 经典双手闭合并等待 */
  bool closeBothAndWait(double ratio = 1.0, double tolerance_rad = 0.2,
                        std::chrono::milliseconds timeout = std::chrono::seconds(3));

  /**
   * @brief 经典夹取：全速闭合 + 轮询 |torque|
   * @param max_torque_nm <=0 时用 config.grasp_torque_limit_nm
   * @return TorqueLimit=碰到物体；PositionReached=空夹合到底；Timeout=超时
   */
  GraspFeedback graspAndWait(Side side, double max_torque_nm = -1.0,
                             std::chrono::milliseconds timeout = std::chrono::seconds(5));

  /** @brief 经典双手夹取（左右独立判力矩） */
  GraspFeedback graspBothAndWait(double max_torque_nm = -1.0,
                                 std::chrono::milliseconds timeout = std::chrono::seconds(5));

  // =========================================================================
  // Soft 接口 — ros2_170D 软力矩（斜率 + kp/kd 反算，不删经典接口）
  // =========================================================================

  /** @brief Soft 模式张开（启用 soft_slew_rate + applySoftTorqueLimit） */
  void openSoft(Side side);

  /** @brief Soft 模式双手张开 */
  void openSoftBoth();

  /** @brief Soft 模式闭合，ratio 0~1 */
  void closeSoft(Side side, double ratio = 1.0);

  /** @brief Soft 模式双手闭合 */
  void closeSoftBoth(double ratio = 1.0);

  /** @brief Soft 张开并阻塞等待到位 */
  bool openSoftAndWait(Side side, double tolerance_rad = 0.2,
                       std::chrono::milliseconds timeout = std::chrono::seconds(5));

  /** @brief Soft 双手张开并等待 */
  bool openSoftBothAndWait(double tolerance_rad = 0.2,
                           std::chrono::milliseconds timeout = std::chrono::seconds(5));

  /** @brief Soft 闭合并阻塞等待到位 */
  bool closeSoftAndWait(Side side, double ratio = 1.0, double tolerance_rad = 0.2,
                        std::chrono::milliseconds timeout = std::chrono::seconds(5));

  /** @brief Soft 双手闭合并等待 */
  bool closeSoftBothAndWait(double ratio = 1.0, double tolerance_rad = 0.2,
                            std::chrono::milliseconds timeout = std::chrono::seconds(5));

  /**
   * @brief Soft 夹取：closeSoft + 等 PositionReached 或软限位持续触顶
   * @param max_torque_nm <=0 时用 config.soft_torque_limit_nm（仅用于判接触，限位在控制线程）
   */
  GraspFeedback graspSoftAndWait(Side side, double max_torque_nm = -1.0,
                                 std::chrono::milliseconds timeout = std::chrono::seconds(5));

  /** @brief Soft 双手夹取 */
  GraspFeedback graspSoftBothAndWait(double max_torque_nm = -1.0,
                                     std::chrono::milliseconds timeout = std::chrono::seconds(5));

  /** @brief 该侧是否处于 Soft 力矩控制路径（openSoft/closeSoft 等会自动开启） */
  bool isSoftTorqueActive(Side side) const;

  // =========================================================================
  // 状态读取与阻塞等待（通用）
  // =========================================================================

  /** @brief 读单侧最新反馈（线程安全） */
  SideFeedback feedback(Side side) const;

  /**
   * @brief 按角度 (°) 阻塞等待
   * @param target_deg 目标角度
   * @param tolerance_deg 容差 (°)
   */
  bool waitFor(Side side, double target_deg, double tolerance_deg = 0.05,
               std::chrono::milliseconds timeout = std::chrono::seconds(5));

  /**
   * @brief 按关节角 q (rad) 阻塞等待
   * @param target_rad 目标 q
   * @param tolerance_rad 容差 (rad)
   */
  bool waitForRad(Side side, double target_rad, double tolerance_rad = 0.15,
                  std::chrono::milliseconds timeout = std::chrono::seconds(5));

  // =========================================================================
  // 标定
  // =========================================================================

  /**
   * @brief 交互式标定：手动紧闭后按 s + Enter
   * @see https://support.unitree.com/home/zh/dex1-1_gripper/dex1_1
   */
  bool calibrateInteractive();

  /**
   * @brief 标定单侧（需已手动紧闭到机械极限）
   * @param side 左/右
   */
  bool calibrate(Side side);

  // =========================================================================
  // 经典遥操（非阻塞，简单 |torque| 锁）
  // =========================================================================

  /**
   * @brief 开启/关闭遥操模式
   * @note 关闭后由 open/close 等经典接口驱动；Soft 遥操用 setTeleopSoftMode
   */
  void setTeleopMode(bool enabled);

  /** @brief 是否处于经典或 Soft 遥操模式 */
  bool teleopMode() const { return teleop_mode_.load(); }

  /**
   * @brief 经典遥操：ratio 0=全开，1=全合，立即返回
   * @note 自动 setTeleopMode(true)；往闭合且 |torque|≥grasp_torque_limit_nm 时锁 q
   */
  void setTeleopRatio(Side side, double ratio);

  /** @brief 经典遥操双手 */
  void setTeleopBoth(double left_ratio, double right_ratio);

  /** @brief 读经典遥操状态（可随时轮询） */
  TeleopFeedback teleopFeedback(Side side) const;

  // =========================================================================
  // Soft 遥操（非阻塞，ros2 kp/kd 反算限位）
  // =========================================================================

  /** @brief 开启/关闭 Soft 遥操（与经典遥操互斥，后设者优先） */
  void setTeleopSoftMode(bool enabled);

  /** @brief 是否 Soft 遥操模式 */
  bool teleopSoftMode() const { return teleop_soft_mode_.load(); }

  /**
   * @brief Soft 遥操：ratio 0~1，控制线程内 soft_slew + applySoftTorqueLimit
   */
  void setTeleopSoftRatio(Side side, double ratio);

  /** @brief Soft 遥操双手 */
  void setTeleopSoftBoth(double left_ratio, double right_ratio);

  /** @brief 读 Soft 遥操状态（字段含义同 TeleopFeedback） */
  TeleopFeedback teleopSoftFeedback(Side side) const;

private:
  struct SideImpl;

  void controlLoop();
  void setTargetRad(Side side, double target_rad);
  void setSoftTargetRad(Side side, double target_rad);
  bool waitBothRad(double target_rad, double tolerance_rad,
                   std::chrono::milliseconds timeout);
  double resolveGraspTorque(double max_torque_nm) const;
  double resolveSoftTorque(double max_torque_nm) const;
  GraspFeedback graspSideUntil(Side side, double max_torque_nm, double position_tolerance_rad,
                               std::chrono::milliseconds timeout);
  GraspFeedback graspSoftSideUntil(Side side, double max_torque_nm,
                                   double position_tolerance_rad,
                                   std::chrono::milliseconds timeout);
  void syncSoftMotionState(SideImpl &impl);
  void applySoftTorqueLimit(SideImpl &impl);
  void applyTeleopTarget(SideImpl &impl);
  void runSoftControlStep(SideImpl &impl, double dt);
  void updateMotionFilter(SideImpl &impl);
  double ratioToRad(double ratio) const;
  double radToRatio(double q) const;

  Config config_;
  std::map<Side, std::unique_ptr<SideImpl>> sides_;

  mutable std::mutex mutex_;
  std::thread control_thread_;
  std::atomic<bool> connected_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> teleop_mode_{false};
  std::atomic<bool> teleop_soft_mode_{false};
};

}  // namespace gripper

#endif
