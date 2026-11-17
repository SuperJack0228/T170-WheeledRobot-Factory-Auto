#include "internal/runtime.h"
#include "t170/skills/conveyor.h"

#include "t170/telemetry/factory_status.h"
#include "t170/telemetry/dispatch_metrics.h"
#include "Ti5_Arm.h"
#include "t170/common/units.h"
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
int skill_legacy_code(const t170::skills::Result &result) {
    if(result.legacy_status) return *result.legacy_status;
    if(result.native_code) return *result.native_code;
    if(result.error==t170::skills::Error::Cancelled) return -4;
    if(result.error==t170::skills::Error::Device) return -2;
    return result.ok()?0:-32;
}

int go_belt_station(
    const ConveyorStationConfig &c, const char *tag,
    const std::function<int(bool is_right)> &on_hand_placed,
    bool do_right, bool do_left,
    bool skip_ready, int photo_after_arrival_ms)
{
    const char *name = (tag && tag[0]) ? tag : "belt";
    if (g_hw == nullptr || g_hw->waist == nullptr || g_hw->layer3 == nullptr)
    {
        std::cerr << "[orch] " << name << " 失败：腰未初始化\n";
        return -1;
    }
    if (g_hw->arm_r == nullptr || g_hw->arm_l == nullptr)
    {
        std::cerr << "[orch] " << name << " 失败：手臂未初始化\n";
        return -1;
    }
    if (hardware_abort_requested())
        return -4;

    g_hw->last_debug_photo.clear();
    g_hw->last_belt_ok = false;
    g_hw->last_belt_name.clear();
    g_hw->last_belt_message.clear();
    g_hw->last_belt_x = std::numeric_limits<double>::quiet_NaN();
    g_hw->last_belt_y = std::numeric_limits<double>::quiet_NaN();
    g_hw->last_belt_z = std::numeric_limits<double>::quiet_NaN();

    auto aborted = []() {
        return hardware_abort_requested() ||
               (g_hw && g_hw->abort && g_hw->abort->load()) || can_io_faulted();
    };

    std::cout << "[orch] " << name << "：准备tcp → 识别传送带5x5 → "
                 "右手放下松爪后，左手立刻过来；右手同时抬起回准备（不回 home）\n";

    if (skip_ready)
    {
        std::cout << "[orch] " << name << " 路上已经摆好拍照姿态，到站后直接等停稳再拍\n";
        g_photo_ready_station.clear();
    }
    else
    {
        const int ready_rc = go_belt_ready(*g_hw->arm_r, *g_hw->arm_l, c, name);
        if (ready_rc != 0)
            return ready_rc;
    }
    if (aborted())
        return -4;
    const int settle_ms = photo_after_arrival_ms >= 0
                              ? std::max(0, photo_after_arrival_ms)
                              : static_cast<int>(std::lround(
                                    std::max(0.0, c.photo_after_arrival_sec) * 1000.0));
    int wait_ms = settle_ms;
    if (g_station_arrived)
    {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::steady_clock::now() - g_station_arrived_at)
                                 .count();
        wait_ms = static_cast<int>(std::max<int64_t>(0, settle_ms - elapsed));
        std::cout << "[orch] " << name << " 导航完成已 "
                  << elapsed << " ms，再等 " << wait_ms << " ms 后拍照"
                  << "（yaml photo_after_arrival_sec=" << c.photo_after_arrival_sec
                  << "s）\n";
    }
    else
        std::cout << "[orch] " << name << " 未记录到站时刻，停稳 " << settle_ms
                  << " ms 再拍照（yaml photo_after_arrival_sec="
                  << c.photo_after_arrival_sec << "s）\n";
    if (wait_ms > 0)
        hardware_abort_sleep_ms(wait_ms);
    if (aborted())
        return -4;

    if (g_hw->pipeline == nullptr)
    {
        std::cerr << "[orch] belt 无相机，无法识别传送带码\n";
        g_hw->last_belt_message = "无相机";
        return kVisionQrFail;
    }
    SegPoseBridge &bridge = g_hw->pipeline->bridge();
    if (!bridge.belt_engine_ready())
    {
        std::cerr << "[orch] 传送带 ArUco 未加载（检查 config/vision/belt.yaml），料盘引擎未动\n";
        g_hw->last_belt_message = "传送带引擎未加载";
        return kVisionQrFail;
    }

    RealSenseMultiCam &cameras = g_hw->pipeline->cameras();
    if (kDebugVisualize)
        pose_vis_begin_phase(PoseVisLayout::Single, PoseVisPanel::Head, {PoseVisPanel::Head});
    HeadBeltPerception perc = perceive_belt_head(
        cameras, bridge, c.prefer_belt, kDebugVisualize, nullptr, nullptr, c.yolo_roi_enable);
    if (!perc.frame.ok)
    {
        std::cerr << "[orch] belt 头相机取帧失败: " << perc.frame.message << "\n";
        g_hw->last_belt_message = "取帧失败";
        return kVisionQrFail;
    }
    BeltDetectResult det = std::move(perc.belt);
    g_hw->last_belt_ok = det.ok && det.in_robot;
    g_hw->last_belt_name = det.name;
    g_hw->last_belt_message = det.message;
    g_hw->last_debug_photo = det.save_path;
    if (det.in_robot)
    {
        g_hw->last_belt_x = det.origin_x;
        g_hw->last_belt_y = det.origin_y;
        g_hw->last_belt_z = det.origin_z;
    }

    std::cout << std::fixed << std::setprecision(4)
              << "[orch] belt 识别 " << (det.ok ? "OK" : "FAIL")
              << " name=" << det.name << " ids=";
    for (size_t i = 0; i < det.used_ids.size(); ++i)
        std::cout << (i ? "," : "") << det.used_ids[i];
    std::cout << " reproj=" << det.reproj_px << "px";
    if (det.in_robot)
        std::cout << " 原点基座=(" << det.origin_x << "," << det.origin_y << ","
                  << det.origin_z << ")";
    else
        std::cout << " 无基座坐标";
    std::cout << " " << det.message << "\n";
    if (!det.save_path.empty())
        std::cout << "[orch] belt 标注图 " << det.save_path << "\n";

    if (!det.ok || !det.in_robot)
    {
        g_hw->last_belt_message = "二维码解算失败";
        std::cerr << "[orch] belt 未拿到传送带坐标（二维码解算失败），手臂停在准备姿态，不往码上走\n";
        return kVisionQrFail;
    }
    if (aborted())
        return -4;

    apply_fixed_belt_origin_z(det, c, name);
    if (det.in_robot)
        g_hw->last_belt_z = det.origin_z;

    if (c.yolo_roi_enable)
    {
        const BeltPartList parts = detect_belt_parts_full(
            bridge, perc.frame, c, det, name, perc.yolo_ran ? &perc.yolo : nullptr);
        if (!parts.ok)
        {
            g_hw->last_belt_message = parts.message;
            std::cerr << "[orch] " << name << " 放置前检测失败: " << parts.message << "\n";
            return kVisionQrFail;
        }
        std::vector<MarkerQuery> queries;
        if (do_right)
            queries.push_back(MarkerQuery{c.offset_right.x, c.offset_right.y, "右手放置点"});
        if (do_left)
            queries.push_back(MarkerQuery{c.offset_left.x, c.offset_left.y, "左手放置点"});
        const std::vector<int> hit = match_parts_to_queries(queries, parts.parts, 0.03);
        int near = 0;
        for (int a : hit)
        {
            if (a >= 0)
                ++near;
        }
        if (near > 0)
        {
            g_hw->last_belt_ok = false;
            g_hw->last_belt_message = "放置点附近有零件";
            std::cerr << "[orch] " << name << " 放置门禁：±3cm 内配到 " << near
                      << " 个零件，左右手都不放\n";
            return kBeltPlaceOccupied;
        }
        std::cout << "[orch] " << name << " 放置门禁通过：放置点 ±3cm 内没有零件\n";
    }

    const auto &off_r = c.offset_right;
    const auto &off_l = c.offset_left;
    Eigen::Matrix<double, 1, 6> goal_r = move_box::make_belt_goal_right(det, c);
    Eigen::Matrix<double, 1, 6> goal_l = move_box::make_belt_goal_left(det, c);
    if (std::abs(c.place_base_z_bias_m) > 1e-9)
    {
        goal_r(2) += c.place_base_z_bias_m;
        goal_l(2) += c.place_base_z_bias_m;
        std::cout << std::fixed << std::setprecision(4)
                  << "[orch] belt 放置松手在基座 Z 再偏 " << c.place_base_z_bias_m
                  << " m，左右手都加\n";
    }
    std::cout << std::fixed << std::setprecision(4)
              << "[orch] belt 放置 原点=(" << det.origin_x << "," << det.origin_y << ","
              << det.origin_z << ") offset_识别系 右=(" << off_r.x << "," << off_r.y << ","
              << off_r.z << ") 左=(" << off_l.x << "," << off_l.y << "," << off_l.z
              << ") have_axes=" << (det.have_axes ? "yes" : "no") << "\n";
    if (det.have_axes)
        std::cout << "[orch] belt 码轴 基座 +X=(" << det.ax_x << "," << det.ax_y << ","
                  << det.ax_z << ") +Y=(" << det.ay_x << "," << det.ay_y << "," << det.ay_z
                  << ") +Z=(" << det.az_x << "," << det.az_y << "," << det.az_z << ")\n";
    std::cout << "[orch] belt 下发目标 右=(" << goal_r(0) << "," << goal_r(1) << ","
              << goal_r(2) << ") 左=(" << goal_l(0) << "," << goal_l(1) << "," << goal_l(2)
              << ") 右手松爪后左手立刻过来，不等右手回准备\n";
    std::cout.flush();

    auto place_arrive_and_open = [&](bool is_right) -> int {
        if(is_right && right_arm_motors_locked()) return 0;
        if(!g_hw->g) return -1;
        t170::skills::ConveyorContext context{*g_hw->g,*g_hw->arm_r,*g_hw->arm_l,c,{}};
        t170::skills::ConveyorPlaceRequest request;
        request.hand=is_right?t170::skills::Hand::Right:t170::skills::Hand::Left;
        request.goal=is_right?goal_r:goal_l;request.metadata.workcell_id=c.prefer_belt;
        request.initial_part=t170::skills::PartState::Holding;context.hooks.log_gripper=log_gripper_qs;
        context.hooks.on_released=[&]{clear_last_grasped(is_right,!is_right);};
        const auto result=t170::skills::execute_conveyor_place(context,request,{aborted,{}});
        std::cout<<"[skill_result] "<<t170::skills::to_json(result)<<'\n';
        return skill_legacy_code(result);
    };

    auto lift_and_retract = [&](bool is_right) -> int {
        if(is_right && right_arm_motors_locked()) return 0;
        if(!g_hw->g) return -1;
        t170::skills::ConveyorContext context{*g_hw->g,*g_hw->arm_r,*g_hw->arm_l,c,{}};
        t170::skills::ConveyorRetreatRequest request;
        request.hand=is_right?t170::skills::Hand::Right:t170::skills::Hand::Left;
        request.after_place=true;request.metadata.workcell_id=c.prefer_belt;
        request.initial_part=t170::skills::PartState::Empty;
        const auto result=t170::skills::execute_conveyor_retreat(context,request,{aborted,{}});
        std::cout<<"[skill_result] "<<t170::skills::to_json(result)<<'\n';
        return skill_legacy_code(result);
    };

    if (!do_right && !do_left)
    {
        std::cout << "[orch] " << name << " 这次没有要放的手，跳过放置动作\n";
        return 0;
    }

    int right_back_rc = 0;
    int right_note_rc = 0;
    std::thread th_r;
    const bool run_right = do_right && !right_arm_motors_locked();
    // 调度完成上报：最后放的那只手收回到准备位后再发 DONE（通常是左手）。
    const bool last_hand_is_left = do_left;
    const bool last_hand_is_right = run_right && !do_left;
    if (run_right)
    {
        const int right_rc = place_arrive_and_open(true);
        if (right_rc != 0)
            return right_rc;
        th_r = std::thread([&]() {
            right_back_rc = lift_and_retract(true);
            if (right_back_rc != 0)
                return;
            if (last_hand_is_right && on_hand_placed)
                right_note_rc = on_hand_placed(true);
        });
    }
    else if (!do_right)
        std::cout << "[orch] belt 右手这次不放\n";

    int left_rc = 0;
    int left_back_rc = 0;
    if (do_left)
    {
        if (run_right)
            std::cout << "[orch] belt 右手已放下，左手立刻过来（不等右手抬起回准备）\n";
        left_rc = place_arrive_and_open(false);
        if (left_rc == 0)
        {
            left_back_rc = lift_and_retract(false);
            if (left_back_rc == 0 && last_hand_is_left && on_hand_placed)
            {
                const int note_rc = on_hand_placed(false);
                if (note_rc != 0)
                {
                    if (th_r.joinable())
                        th_r.join();
                    return note_rc;
                }
            }
        }
    }
    else
        std::cout << "[orch] belt 左手这次不放\n";
    if (th_r.joinable())
        th_r.join();
    if (right_note_rc != 0)
        return right_note_rc;
    if (left_rc != 0)
        return left_rc;
    if (left_back_rc != 0)
        return left_back_rc;
    if (right_back_rc != 0)
        return right_back_rc;

    std::cout << "[orch] " << name << " 双手放置结束，停在准备姿态\n";
    return 0;
}

int go_belt_station()
{
    return go_belt_station(g_move_cfg.conveyor, "belt");
}

int go_belt()
{
    const std::string &st = g_move_cfg.chassis.belt_station;
    std::cout << "[orch] belt：先到 " << st << " 再放置\n";
    const int ch_rc = chassis_goto_station(st);
    if (ch_rc != 0)
        return ch_rc;
    if (hardware_abort_requested())
        return -4;
    std::cout << "[orch] belt 底盘已到 " << st << "，到站后再等 "
              << g_move_cfg.conveyor.photo_after_arrival_sec << "s 拍照\n";
    return go_belt_station(g_move_cfg.conveyor, "belt");
}
}
