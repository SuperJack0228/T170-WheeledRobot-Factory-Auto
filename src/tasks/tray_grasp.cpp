#include "internal/runtime.h"
#include "t170/tasks/skill_failure.h"
#include "t170/skills/conveyor.h"

#include "t170/telemetry/factory_status.h"
#include "t170/telemetry/dispatch_metrics.h"
#include "t170/motion/arm.h"
#include "t170/common/units.h"
#include "t170/motion/cartesian_tracking.h"
#include "t170/motion/retreat.h"
#include "t170/motion/servo_handoff.h"
#include "t170/vision/wrist_observer.h"
#include "t170/motion/servo.h"
#include "t170/vision/head_transform.h"
#include "t170/hardware/head.h"
#include "t170/skills/legacy/profiles.h"
#include "t170/skills/legacy/runtime.h"
#include "t170/vision/python_bridge.h"
#include "t170/vision/dl.h"
#include "t170/vision/debug.h"
#include "t170/hardware/device_bus.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <cerrno>
#include <csignal>
#include <mutex>
#include "t170/motion/retreat_gate.h"
#include <sys/select.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using std::vector;
using std::string;
using std::thread;


namespace orch {
namespace {
constexpr int kTrayGripperNotReady = -39;
constexpr double kTrayReadyCloseRatio = 0.2;
constexpr double kTrayReadyToleranceRad = 0.15;

// 下发准备开度后核对编码器。已持件的手保持合爪，不在这里松开。
bool settle_tray_grasp_ready(bool want_right, bool want_left)
{
    if (!want_right && !want_left)
        return true;
    if (g_hw == nullptr || g_hw->g == nullptr)
        return false;
    const double grip_r = std::clamp(kTrayReadyCloseRatio, 0.0, 1.0);
    std::cout << std::fixed << std::setprecision(2)
              << "[grasp] 准备姿态夹爪收到 " << grip_r
              << "（0全开 1全合），下发后核对编码器\n";
    log_gripper_qs("料盘抓取准备夹爪前");
    bool ready = true;
    auto settle = [&](bool is_right) {
        if (is_right && (right_arm_motors_locked() || !want_right))
            return;
        if (!is_right && !want_left)
            return;
        const auto side = is_right ? gripper::Side::Right : gripper::Side::Left;
        const char *which = is_right ? "右手" : "左手";
        const auto &cfg = g_hw->g->config();
        const double target = cfg.open_position_rad +
                              grip_r * (cfg.close_position_rad - cfg.open_position_rad);
        if (!g_hw->g->hasSide(side))
        {
            std::cerr << "[gripper] " << which << "未配对，准备开度未到位\n";
            ready = false;
            return;
        }
        const bool waited = g_hw->g->closeAndWait(
            side, grip_r, kTrayReadyToleranceRad, std::chrono::milliseconds(3000));
        const auto fb = g_hw->g->feedback(side);
        const bool at = waited && fb.have_feedback && fb.io_ok && fb.merror == 0 &&
                        std::isfinite(fb.position_rad) &&
                        std::fabs(fb.position_rad - target) <= kTrayReadyToleranceRad;
        std::cout << std::fixed << std::setprecision(3)
                  << "[gripper] " << which << "准备开度 目标q=" << target
                  << " 实际q=" << fb.position_rad
                  << (at ? " 到位\n" : " 未到位\n");
        if (!at)
        {
            ready = false;
            return;
        }
        clear_last_grasped(is_right, !is_right);
    };
    settle(false);
    settle(true);
    log_gripper_qs("料盘抓取准备夹爪后");
    if (!ready)
        std::cerr << "[grasp] 夹爪未到准备开度，不伸手\n";
    return ready;
}

int refuse_unready_gripper()
{
    const std::string msg = "料盘抓取夹爪未到准备开度，不伸手";
    if (g_hw != nullptr)
    {
        g_hw->last_task_error = msg;
        g_hw->last_grasp_status = kTrayGripperNotReady;
    }
    board_error(msg);
    return kTrayGripperNotReady;
}
} // namespace

int vision_grasp_with_engine(SegEngineId engine_id, bool adaptive_approach_rpy,
                            bool depart_after_grasp, bool clear_tray_when_stay)
{
    g_load_parts_seen = -1;
    g_grasp_departed = false;
    if(g_hw) {g_hw->last_skill_result.reset();g_hw->last_task_error.clear();g_hw->last_grasp_count=0;}
    if (g_hw == nullptr || g_hw->g == nullptr || g_hw->pipeline == nullptr)
        return -1;

    auto reject_configuration=[&](const std::string &error) {
        g_hw->last_skill_result=t170::tasks::configuration_failure(t170::skills::Skill::TrayGrasp,error);
        g_hw->last_task_error=error;std::cerr<<"[grasp_preflight] "<<error<<'\n';return -30;
    };
    if(g_move_cfg.cameras.hand_descent_correct) {
        for(bool right:{false,true}) {
            if((right && right_arm_motors_locked()) || !g_hw->g->hasSide(right?gripper::Side::Right:gripper::Side::Left))continue;
            const int hand=right?1:0;
            if(g_move_cfg.visual_servo.profiles[hand].mode!="geometry" &&
               g_move_cfg.visual_servo.calibrated_tray_grasp[hand].empty())continue;
            std::array<double,16> eye;std::string error;
            if(!load_cam2robot_matrix(right?default_right_hand_to_robot_yaml_path():default_left_hand_to_robot_yaml_path(),eye,error)) {
                return reject_configuration(error);
            }
        }
    }

    std::array<double, 16> cam2robot{};
    std::string err;
    if (!load_head_cam2robot(cam2robot, err) &&
        !load_cam2robot_matrix(default_camera_to_robot_yaml_path(), cam2robot, err))
    {
        std::cerr << "[orch] 读 cam2robot 失败: " << err << std::endl;
        g_hw->last_task_error=err;
        return -3;
    }

    auto should_abort = []() {
        return hardware_abort_requested() ||
               (g_hw && g_hw->abort && g_hw->abort->load()) || can_io_faulted();
    };

    const std::string here = chassis_location_station();
    const bool from_tray2 = station_name_matches(here, g_move_cfg.chassis.tray2_station);
    const bool from_belt2 =
        !from_tray2 && station_name_matches(here, g_move_cfg.chassis.out_station);
    // 料盘二回料盘一：先升腰出发。站名变成上料点、底盘尚未停稳时就开始降腰。
    // 传送带二回料盘一：出发 2s 后路上降腰摆手。
    const bool lower_on_station_name = from_tray2;
    if (from_tray2)
    {
        std::cout << "[grasp] 现在在料盘二，先升腰，再去 "
                  << g_move_cfg.chassis.tray_station << "，到上料点就开始降腰，不等停稳\n";
        const int up_rc = home_waist_upright();
        if (up_rc != 0)
            return up_rc;
        if (should_abort())
            return -4;
    }
    else if (from_belt2)
    {
        std::cout << "[grasp] 现在在传送带二，去 "
                  << g_move_cfg.chassis.tray_station
                  << "；底盘一出发等 2s 开始降腰并摆手部姿态"
                  << (move_box::tray1_far_band_active() ? "（远三排记忆已开）\n" : "\n");
    }
    else
    {
        std::cout << "[grasp] 去料盘 " << g_move_cfg.chassis.tray_station
                  << "，路上同时摆头、手臂、开爪和下腰"
                  << (move_box::tray1_far_band_active() ? "（远三排记忆已开）\n" : "\n");
    }
    board_step("去料盘一");
    int prep_rc = 0;
    int travel_rc = 0;
    std::atomic<bool> early_waist_started{false};
    std::atomic<int> early_waist_rc{0};
    std::thread early_waist;
    {
        auto lower_waist = [&]() {
            bool expected = false;
            if (!early_waist_started.compare_exchange_strong(expected, true))
                return;
            early_waist = std::thread([&]() {
                std::cout << "[grasp] 底盘已到 " << g_move_cfg.chassis.tray_station
                          << "，开始降腰，不等停稳\n" << std::flush;
                if (g_hw->waist == nullptr || g_hw->layer3 == nullptr ||
                    !move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
                {
                    early_waist_rc = hardware_abort_requested() ? -4 : -5;
                    return;
                }
                if (move_box::tray1_far_band_active())
                {
                    move_box::waist_layer3_ensure_far_row_forward(
                        *g_hw->waist, *g_hw->layer3, g_move_cfg.waist.far_row_forward_m);
                    enable_head_far_pitch();
                }
                move_box::waist_layer3_set_synced(true);
                early_waist_rc = hardware_abort_requested() ? -4 : 0;
            });
        };
        ChassisTravel travel(g_move_cfg.chassis.tray_station, false,
                             lower_on_station_name ? lower_waist : std::function<void()>{});
        auto prep_upper_and_waist = [&](bool do_waist) -> int {
            const bool far_mem = move_box::tray1_far_band_active();
            const int head_rc =
                far_mem ? enable_head_far_pitch() : enable_head_calib_pose();
            if (head_rc != 0)
                return head_rc;
            if (!move_box::move_arms_to_standby(*g_hw->arm_r, *g_hw->arm_l))
            {
                std::cerr << "[grasp] 进入时回到 standby 失败\n";
                return should_abort() ? -4 : -5;
            }
            if (should_abort())
                return -4;
            if (!settle_tray_grasp_ready(!right_arm_motors_locked(), true))
                return refuse_unready_gripper();
            if (should_abort())
                return -4;
            if (!do_waist)
                return 0;
            if (g_hw->waist == nullptr || g_hw->layer3 == nullptr)
            {
                std::cerr << "[grasp] 腰未初始化，拒绝进入抓取\n";
                return -1;
            }
            if (!move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
            {
                std::cerr << "[grasp] 腰未到 ready 起始位，放弃抓取\n";
                return should_abort() ? -4 : -5;
            }
            if (far_mem)
            {
                std::cout << "[grasp] 远三排记忆：路上腰再前伸到远排位\n";
                move_box::waist_layer3_ensure_far_row_forward(
                    *g_hw->waist, *g_hw->layer3, g_move_cfg.waist.far_row_forward_m);
            }
            move_box::waist_layer3_set_synced(true);
            return should_abort() ? -4 : 0;
        };

        if (from_belt2)
        {
            std::cout << "[grasp] 底盘已出发，等 2s 后降腰摆手\n";
            hardware_abort_sleep_ms(2000);
            if (should_abort())
                prep_rc = -4;
            else
                prep_rc = prep_upper_and_waist(true);
        }
        else if (lower_on_station_name)
        {
            prep_rc = prep_upper_and_waist(false);
            if (prep_rc == 0)
                std::cout << "[grasp] 手臂先摆好，腰等底盘站名变为 "
                          << g_move_cfg.chassis.tray_station << " 再降\n";
        }
        else
            prep_rc = prep_upper_and_waist(true);
        travel_rc = travel.join();
    }
    if (early_waist.joinable())
        early_waist.join();
    if (should_abort())
        return -4;
    if (travel_rc != 0)
        return travel_rc;
    if (prep_rc != 0)
        return prep_rc;
    if (lower_on_station_name && !early_waist_started.load())
    {
        std::cout << "[grasp] 底盘停稳前没有提前降腰，现在降\n";
        if (g_hw->waist == nullptr || g_hw->layer3 == nullptr ||
            !move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
        {
            std::cerr << "[grasp] 到站后降腰失败，放弃抓取\n";
            return should_abort() ? -4 : -5;
        }
        if (move_box::tray1_far_band_active())
        {
            move_box::waist_layer3_ensure_far_row_forward(
                *g_hw->waist, *g_hw->layer3, g_move_cfg.waist.far_row_forward_m);
            enable_head_far_pitch();
        }
        move_box::waist_layer3_set_synced(true);
    }
    if (early_waist_started.load() && early_waist_rc.load() != 0)
    {
        std::cerr << "[grasp] 到站降腰失败，放弃抓取\n";
        return early_waist_rc.load();
    }
    if (should_abort())
        return -4;
    if (!lower_on_station_name)
    {
        std::cout << "[grasp] 已在 " << g_move_cfg.chassis.tray_station << "，停稳 0.5s 再抓\n";
        hardware_abort_sleep_ms(500);
        if (should_abort())
            return -4;
    }
    else
        std::cout << "[grasp] 已在 " << g_move_cfg.chassis.tray_station << "，腰和底盘都到位，直接抓\n";
    board_step("料盘一抓取");

    

    g_load_parts_seen = -1;

    const bool clear_tray = !depart_after_grasp && clear_tray_when_stay;
    const int kMaxRounds = clear_tray ? 80 : 40;
    constexpr int kMaxNoneStreak = 2;
    int grasped_count = 0;
    int none_streak = 0;
    int empty_streak_r = 0;
    int empty_streak_l = 0;
    const int max_empty_retry = g_move_cfg.gripper_check.max_empty_retry;
    bool holding_r = false;
    bool holding_l = false;
    g_hw->last_grasp_count = 0;
    g_hw->last_grasped_r = false;
    g_hw->last_grasped_l = false;

    const auto both_holding = [&]() -> bool {
        return right_arm_motors_locked() ? holding_l : (holding_r && holding_l);
    };

    if (clear_tray)
        std::cout << "[grasp] 开始清空料盘：每轮收手后松爪，同时拍照抓下一轮，抓完停在原地"
                  << (right_arm_motors_locked() ? "（右臂锁定，只抓左手）\n" : "\n");
    else
        std::cout << "[grasp] 开始循环：夹后保持合爪；双手都有件才结束，缺的再去拿"
                  << (depart_after_grasp ? "" : "；抓完停在原地不走")
                  << (right_arm_motors_locked() ? "（右臂锁定，只要求左手持件）\n" : "\n");

    std::thread chassis_th;
    std::atomic<int> chassis_rc{0};
    bool chassis_started = false;
    struct ChassisJoin
    {
        std::thread &th;
        ~ChassisJoin()
        {
            if (th.joinable())
                th.join();
        }
    } chassis_join{chassis_th};
    const std::string next_station = g_move_cfg.chassis.belt_station;
    auto start_chassis = [&](const char *why) {
        if (chassis_started)
            return;
        chassis_started = true;
        std::cout << "[grasp] " << why << "，底盘立刻去 " << next_station << std::endl;
        if (g_on_grasp_depart)
            g_on_grasp_depart();
        chassis_th = std::thread([&]() {
            try
            {
                chassis_rc = chassis_goto_station(next_station);
            }
            catch (const std::exception &ex)
            {
                std::cerr << "[grasp] 底盘导航失败: " << ex.what() << "\n";
                chassis_rc = -6;
            }
        });
    };
    int last_rc = 1;
    for (int round = 1; round <= kMaxRounds; ++round)
    {
        if (should_abort())
        {
            g_hw->last_grasp_count = grasped_count;
            g_hw->last_grasp_status = -4;
            std::cout << "[grasp] 循环中止 已持件 右=" << (holding_r ? 1 : 0)
                      << " 左=" << (holding_l ? 1 : 0) << "\n";
            return -4;
        }

        if (!clear_tray && both_holding())
        {
            last_rc = 0;
            std::cout << "[grasp] 双手已持件，结束抓取循环\n";
            break;
        }

        std::cout << "\n======== [grasp] 第 " << round << " 轮 持件 右="
                  << (holding_r ? 1 : 0) << " 左=" << (holding_l ? 1 : 0)
                  << " ========\n";
        const bool open_r = !right_arm_motors_locked() && (clear_tray || !holding_r);
        const bool open_l = clear_tray || !holding_l;
        if (!settle_tray_grasp_ready(open_r, open_l))
        {
            g_hw->last_grasp_count = grasped_count;
            return refuse_unready_gripper();
        }
        if (should_abort())
        {
            g_hw->last_grasp_count = grasped_count;
            g_hw->last_grasp_status = -4;
            return -4;
        }
        board_activity("料盘一第 " + std::to_string(round) + " 轮：拍照识别并抓取");
        move_box::GraspToStandbyResult r = move_box::run_grasp_to_standby(
            *g_hw->g,
            *g_hw->arm_r,
            *g_hw->arm_l,
            *g_hw->waist,
            *g_hw->layer3,
            g_hw->pipeline->cameras(),
            g_hw->pipeline->bridge(),
            cam2robot,
            err,
            should_abort,
            engine_id,
            adaptive_approach_rpy,
            clear_tray ? false : holding_r,
            clear_tray ? false : holding_l,
            {}, !clear_tray,
            depart_after_grasp ? std::function<void()>{[&]{ start_chassis("腰已站起"); }}
                               : std::function<void()>{});

        if (g_load_parts_seen < 0 && r.zone_count_ok)
        {
            g_load_parts_seen = r.zone_right + r.zone_left;
            std::cout << "[grasp] 选孔与计数同一张照片，开抓前工作区毛坯 "
                      << g_load_parts_seen
                      << " 件（右=" << r.zone_right << " 左=" << r.zone_left << "）\n";
        }
        if (r.zone_count_ok)
            board_tray1(r.zone_right - (r.grasped_r ? 1 : 0), r.zone_left - (r.grasped_l ? 1 : 0));
        board_tray1_taken((r.grasped_r ? 1 : 0) + (r.grasped_l ? 1 : 0));

        if (!clear_tray)
        {
            holding_r = holding_r || r.grasped_r;
            holding_l = holding_l || r.grasped_l;
            grasped_count = (holding_r ? 1 : 0) + (holding_l ? 1 : 0);
        }
        g_hw->last_grasped_r = holding_r;
        g_hw->last_grasped_l = holding_l;
        g_hw->last_grasp_status = static_cast<int>(r.status);
        g_hw->last_skill_result=r.execution;
        if(r.execution) {
            const int failure=t170::tasks::terminal_skill_failure(*r.execution);
            if(failure) {
                g_hw->last_grasped_r=holding_r || r.grasped_r;g_hw->last_grasped_l=holding_l || r.grasped_l;
                g_hw->last_grasp_count=grasped_count+(clear_tray?int(r.grasped_r)+int(r.grasped_l):0);
                g_hw->last_task_error=r.execution->message+"，阶段="+t170::skills::name(r.execution->failure_stage.value_or(r.execution->stage))+"，原始码="+std::to_string(r.execution->native_code.value_or(failure));
                if(r.execution->error==t170::skills::Error::Servo)for(int hand=0;hand<2;++hand) {
                    if(!r.execution->hands[hand].requested)continue;
                    const auto reason=wrist_servo::last_failure(hand);
                    if(!reason.empty())g_hw->last_task_error+=(hand?"；右手：":"；左手：")+reason;
                }
                std::cerr<<"[grasp] 终止当前任务，不重试："<<g_hw->last_task_error<<'\n';
                return failure;
            }
        }

        if (r.status == move_box::GraspToStandbyStatus::ReleaseFail) {
            std::cerr << "[tray_place] 张爪未确认，停止任务，保留已确认放置的侧\n";
            return -38;
        }
        if (r.status == move_box::GraspToStandbyStatus::MotionFail) {
            std::cerr << "[retract] 收手未完成，停止当前任务，保留已持件/已放置状态\n";
            return -37;
        }
        if (r.status == move_box::GraspToStandbyStatus::Aborted)
        {
            g_hw->last_grasp_count = grasped_count;
            std::cout << "[grasp] 循环中止 已持件 右=" << (holding_r ? 1 : 0)
                      << " 左=" << (holding_l ? 1 : 0) << "\n";
            return -4;
        }

        if(g_move_cfg.visual_servo.require_dual) {
            // One simultaneous attempt only: do not turn a dual-hand validation into serial retries.
            last_rc=r.grasped_r && r.grasped_l && r.status==move_box::GraspToStandbyStatus::Ok ? 0 : -34;
            std::cout << "[servo] 双手单次验证结束：右=" << r.grasped_r << " 左=" << r.grasped_l
                      << " code=" << last_rc
                      << " root_right=" << wrist_servo::last_failure(1)
                      << " root_left=" << wrist_servo::last_failure(0)
                      << "，不自动补抓\n";
            break;
        }

        if (r.empty_r || r.empty_l)
            board_note("warn", std::string("料盘一空抓") + (r.empty_r ? " 右手" : "") +
                                   (r.empty_l ? " 左手" : "") + "，不计数，原地重拍重抓");
        empty_streak_r = r.empty_r ? empty_streak_r + 1 : (r.grasped_r ? 0 : empty_streak_r);
        empty_streak_l = r.empty_l ? empty_streak_l + 1 : (r.grasped_l ? 0 : empty_streak_l);
        if (empty_streak_r > max_empty_retry || empty_streak_l > max_empty_retry)
        {
            const bool right = empty_streak_r > max_empty_retry;
            const std::string msg = std::string("料盘一") + (right ? "右手" : "左手") + "连续空抓 " +
                                    std::to_string(right ? empty_streak_r : empty_streak_l) +
                                    " 次，停止重抓，检查夹爪和零件";
            std::cerr << "[grasp] " << msg << "\n";
            board_error(msg);
            last_rc = 2;
            break;
        }

        if (r.status == move_box::GraspToStandbyStatus::Ok || r.grasped_r || r.grasped_l)
        {
            none_streak = 0;
            if (clear_tray)
            {
                const int n = (r.grasped_r ? 1 : 0) + (r.grasped_l ? 1 : 0);
                grasped_count += n;
                g_hw->last_grasp_count = grasped_count;
                std::cout << "[grasp] 第 " << round << " 轮收手完成，松开并开始下一轮拍照"
                          << " 本轮抓=" << n << " 累计=" << grasped_count << "\n";
                if (r.grasped_r && !right_arm_motors_locked())
                    g_hw->g->close(gripper::Side::Right, 0.2);
                if (r.grasped_l)
                    g_hw->g->close(gripper::Side::Left, 0.2);
                clear_last_grasped(r.grasped_r, r.grasped_l);
                last_rc = 0;
                continue;
            }
            std::cout << "[grasp] 第 " << round << " 轮完成 本轮新抓"
                      << " 右=" << (r.grasped_r ? 1 : 0)
                      << " 左=" << (r.grasped_l ? 1 : 0)
                      << " 现持件 右=" << (holding_r ? 1 : 0)
                      << " 左=" << (holding_l ? 1 : 0)
                      << "，保持合爪";
            if (both_holding())
            {
                std::cout << "，双手已齐\n";
                last_rc = 0;
                break;
            }
            std::cout << "，缺件继续补抓\n";
            last_rc = 0;
            continue;
        }

        if (r.status == move_box::GraspToStandbyStatus::NoTarget)
        {
            if (!r.zone_count_ok)
            {
                g_hw->last_tray_message = "二维码解算失败";
                std::cerr << "[grasp] 料盘二维码解算失败，本轮不伸手\n";
                last_rc = kVisionQrFail;
                break;
            }
            if (clear_tray)
            {
                std::cout << "[grasp] 料盘已无待抓零件，累计 " << grasped_count
                          << " 件，结束循环\n";
                last_rc = grasped_count > 0 ? 0 : 1;
                break;
            }
            if (both_holding())
            {
                last_rc = 0;
                break;
            }
            std::cout << "[grasp] 工作区已无待抓目标，仍缺件"
                      << " 右=" << (holding_r ? 1 : 0)
                      << " 左=" << (holding_l ? 1 : 0) << "，结束循环\n";
            last_rc = (holding_r || holding_l) ? 2 : 1;
            break;
        }

        ++none_streak;
        std::cerr << "[grasp] 第 " << round << " 轮未夹到 code="
                  << static_cast<int>(r.status) << " 连续失败=" << none_streak
                  << " 持件 右=" << (holding_r ? 1 : 0)
                  << " 左=" << (holding_l ? 1 : 0) << "\n";
        std::cout << "[grasp] 手还在外面，先收回旁边，收回之后才允许下一次头相机拍照\n";
        if (!move_box::move_arms_to_standby(*g_hw->arm_r, *g_hw->arm_l))
        {
            std::cerr << "[grasp] 收回失败，不再拍照\n";
            last_rc = should_abort() ? -4 : -5;
            break;
        }
        if (none_streak >= kMaxNoneStreak)
        {
            std::cerr << "[grasp] 连续 " << none_streak << " 轮未夹到，退出循环\n";
            last_rc = both_holding() ? 0 : ((holding_r || holding_l) ? 2 : 2);
            break;
        }
        last_rc = 2;
    }

    g_hw->last_grasp_count = grasped_count;
    g_hw->last_grasp_status = last_rc;
    std::cout << "\n[grasp] 循环结束 持件 右=" << (holding_r ? 1 : 0)
              << " 左=" << (holding_l ? 1 : 0)
              << " 共 " << grasped_count << " 件"
              << (clear_tray ? "，夹爪松开" : "，夹爪保持合上") << std::endl;
    if (clear_tray)
    {
        if (!right_arm_motors_locked())
            g_hw->g->closeAndWait(
                gripper::Side::Right, 0.2, 0.2, std::chrono::milliseconds(800));
        g_hw->g->closeAndWait(
            gripper::Side::Left, 0.2, 0.2, std::chrono::milliseconds(800));
        std::cout << "[grasp] 料盘抓取结束，停在原地，底盘不走" << std::endl;
        return last_rc;
    }
    if ((holding_r || holding_l) && depart_after_grasp)
    {
        board_step("去传送带一");
        start_chassis("收手已完成");
        std::cout << "[grasp] 底盘已出发，腰直接到传送带拍照前伸，不再先收回再伸出"
                  << std::endl;
        waist_rotate_deg(0.0, 30.0);
        const int pose_rc = conveyor_waist_and_head(g_move_cfg.conveyor, "grasp_depart");
        if (chassis_th.joinable())
            chassis_th.join();
        if (pose_rc != 0)
            return pose_rc;
        if (chassis_rc.load() != 0)
            return chassis_rc.load();
        g_grasp_departed = true;
        g_photo_ready_station = next_station;
        std::cout << "[grasp] 腰和底盘已结束，已在 " << next_station
                  << "，拍照姿态已摆好" << std::endl;
    }
    else if (holding_r || holding_l)
        std::cout << "[grasp] 抓完停在原地，底盘不走，夹爪保持合上" << std::endl;
    return last_rc;
}

int vision_grasp_existing_pipeline(bool depart_after_grasp)
{
    return vision_grasp_with_engine(SegEngineId::Default, false, depart_after_grasp);
}
}
