#include "internal/runtime.h"
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
static int go_belt_grasp_attempt(
    bool from_place, const ConveyorStationConfig &c, const std::string &station, const char *tag,
    int only_hand, const std::function<int(bool is_right)> &on_grasped,
    bool photo_settle, const std::string &depart_after_right,
    const std::function<void(bool is_right)> &on_lift, int &empty_mask)
{
    empty_mask = 0;
    // 新一轮抓取先清掉上次路上降腰的标记，避免误跳过料盘二准备。
    g_tray2_place_prep_done = false;
    struct BeltDepartGuard
    {
        ~BeltDepartGuard()
        {
            take_belt_depart();
            take_tray2_waist();
        }
    } belt_depart_guard;
    if (g_hw == nullptr || g_hw->g == nullptr || g_hw->waist == nullptr || g_hw->layer3 == nullptr)
    {
        std::cerr << "[orch] belt_grasp 失败：腰未初始化\n";
        return -1;
    }
    if (g_hw->arm_r == nullptr || g_hw->arm_l == nullptr)
    {
        std::cerr << "[orch] belt_grasp 失败：手臂未初始化\n";
        return -1;
    }
    if (hardware_abort_requested())
        return -4;

    g_photo_ready_station.clear();
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

    const char *name = (tag && tag[0]) ? tag : "belt_grasp";
    if (from_place)
        std::cout << "[orch] " << name << "：放置后已在准备 tcp，不转腰，直接拍照 → 抓取\n";
    else
        std::cout << "[orch] " << name << "：先到 " << station
                  << " → 腰/头到位 → 拍照 → 再动手臂（不转腰）\n";

    if (!from_place)
    {
        const int ch_rc = chassis_goto_station(station);
        if (ch_rc != 0)
            return ch_rc;
        if (aborted())
            return -4;
        std::cout << "[orch] " << name << " 底盘已到 " << station
                  << "，腰头到位后停稳 " << c.grasp_photo_settle_sec << "s 再拍照\n";
        const int wh = conveyor_waist_and_head(c, name);
        if (wh != 0)
            return wh;
        if (aborted())
            return -4;
    }

    const int grasp_settle_ms = static_cast<int>(
        std::lround(std::max(0.0, c.grasp_photo_settle_sec) * 1000.0));
    if (photo_settle)
    {
        std::cout << "[orch] belt_grasp 不转腰，停稳 " << c.grasp_photo_settle_sec
                  << "s 再拍照；grasp_offset 按码坐标系，允许过中轴\n";
        if (grasp_settle_ms > 0)
            hardware_abort_sleep_ms(grasp_settle_ms);
    }
    else
        std::cout << "[orch] belt_grasp 这一只手底盘没动过，不再停 "
                  << c.grasp_photo_settle_sec << "s，直接拍照\n";
    if (aborted())
        return -4;

    if (g_hw->pipeline == nullptr)
    {
        std::cerr << "[orch] belt_grasp 无相机，无法识别传送带码\n";
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
    const double grasp_off_l[3] = {
        c.grasp_offset_left.x, c.grasp_offset_left.y, c.grasp_offset_left.z};
    const double grasp_off_r[3] = {
        c.grasp_offset_right.x, c.grasp_offset_right.y, c.grasp_offset_right.z};
    HeadBeltPerception perc = perceive_belt_head(
        cameras, bridge, c.prefer_belt, kDebugVisualize, grasp_off_l, grasp_off_r,
        c.yolo_roi_enable);
    if (!perc.frame.ok)
    {
        std::cerr << "[orch] belt_grasp 头相机取帧失败: " << perc.frame.message << "\n";
        g_hw->last_belt_message = "取帧失败";
        return kVisionQrFail;
    }
    const CameraFrameData &frame = perc.frame;
    BeltDetectResult det = std::move(perc.belt);
    int hand = only_hand;
    // 窗口判定后可能把 hand 从 -1 收成 0/1；先按传入值估抓点需求，判定后再收紧。
    auto need_l_fn = [&]() { return hand != 1; };
    auto need_r_fn = [&]() { return hand != 0; };
    g_hw->last_belt_ok =
        det.ok && (!need_l_fn() || det.have_grasp_l) && (!need_r_fn() || det.have_grasp_r);
    g_hw->last_belt_name = det.name;
    g_hw->last_belt_message = det.message;
    g_hw->last_debug_photo = det.save_path;
    if (det.have_grasp_l)
    {
        g_hw->last_belt_x = det.grasp_lx;
        g_hw->last_belt_y = det.grasp_ly;
        g_hw->last_belt_z = det.grasp_lz;
    }
    else if (det.in_robot)
    {
        g_hw->last_belt_x = det.origin_x;
        g_hw->last_belt_y = det.origin_y;
        g_hw->last_belt_z = det.origin_z;
    }

    std::cout << std::fixed << std::setprecision(4)
              << "[orch] belt_grasp 识别 " << (det.ok ? "OK" : "FAIL")
              << " name=" << det.name << " ids=";
    for (size_t i = 0; i < det.used_ids.size(); ++i)
        std::cout << (i ? "," : "") << det.used_ids[i];
    std::cout << " reproj=" << det.reproj_px << "px";
    if (det.in_robot)
        std::cout << " 原点基座=(" << det.origin_x << "," << det.origin_y << ","
                  << det.origin_z << ")";
    else
        std::cout << " 无基座坐标";
    if (det.have_grasp_l)
        std::cout << " 左抓=(" << det.grasp_lx << "," << det.grasp_ly << "," << det.grasp_lz << ")";
    if (det.have_grasp_r)
        std::cout << " 右抓=(" << det.grasp_rx << "," << det.grasp_ry << "," << det.grasp_rz << ")";
    if (!det.have_grasp_l && !det.have_grasp_r)
        std::cout << " 无抓取点";
    if (det.have_axes)
        std::cout << " 板+Y基座=(" << det.ay_x << "," << det.ay_y << "," << det.ay_z << ")";
    std::cout << " " << det.message << "\n";
    if (!det.save_path.empty())
        std::cout << "[orch] belt_grasp 标注图 " << det.save_path << "\n";

    if (!det.ok)
    {
        g_hw->last_belt_message = "二维码解算失败";
        std::cerr << "[orch] belt_grasp 二维码解算失败，不往码上走\n";
        return kVisionQrFail;
    }
    apply_fixed_belt_origin_z(det, c, name);
    if (det.have_grasp_l)
        g_hw->last_belt_z = det.grasp_lz;
    else if (det.in_robot)
        g_hw->last_belt_z = det.origin_z;

    if (c.yolo_roi_enable)
    {
        const BeltPartList parts = detect_belt_parts_full(
            bridge, frame, c, det, name, perc.yolo_ran ? &perc.yolo : nullptr);
        if (!parts.ok)
        {
            g_hw->last_belt_ok = false;
            g_hw->last_belt_message = parts.message;
            std::cerr << "[orch] " << name << " 抓取前检测失败: " << parts.message << "\n";
            return kVisionQrFail;
        }
        const BeltGraspWindow win = classify_belt_grasp_window(parts.parts, name);
        const bool grasp_right = win.right;
        const bool grasp_left = win.left;
        if (!grasp_right && !grasp_left)
        {
            g_hw->last_belt_ok = false;
            g_hw->last_belt_message = "抓取窗口内没有料";
            std::cerr << "[orch] " << name << " 码系抓取窗口内没有零件，两手都不抓\n";
            return kBeltNoPartVision;
        }
        if (hand < 0)
        {
            if (grasp_right && grasp_left)
                hand = -1;
            else if (grasp_right)
                hand = 1;
            else
                hand = 0;
        }
        else if (hand == 1 && !grasp_right)
        {
            if (g_hw->last_grasped_l && win.count > 0)
                std::cout << "[orch] " << name << " 左手已夹住，不松开，窗口内还有零件，只补右手\n";
            else
                return grasp_left ? kBeltHandSkip : kBeltNoPartVision;
        }
        else if (hand == 0 && !grasp_left)
        {
            // 右手已经夹住时，件数常被判成「只伸右手」。这只手不再动，有料就补左手。
            if (g_hw->last_grasped_r && win.count > 0)
                std::cout << "[orch] " << name << " 右手已夹住，不松开，窗口内还有零件，只补左手\n";
            else
                return grasp_right ? kBeltHandSkip : kBeltNoPartVision;
        }
        std::cout << "[orch] " << name << " 抓取窗口 右=" << (grasp_right ? "有" : "无")
                  << " 左=" << (grasp_left ? "有" : "无")
                  << (hand < 0 ? "，双手先右后左" : hand == 1 ? "，只伸右手" : "，只伸左手")
                  << "\n";
    }

    if ((need_l_fn() && !det.have_grasp_l) || (need_r_fn() && !det.have_grasp_r))
    {
        g_hw->last_belt_message = "未拿到抓取点";
        std::cerr << "[orch] belt_grasp 未拿到"
                  << (hand < 0 ? "左右" : (hand == 0 ? "左手" : "右手"))
                  << "抓取点，不往码上走\n";
        return kVisionQrFail;
    }
    if (aborted())
        return -4;

    if (from_place)
        std::cout << "[orch] belt_grasp 认码完成，手臂已在放置后的准备 tcp，直接抓\n";
    else
    {
        std::cout << "[orch] belt_grasp 认码完成，手臂到准备 tcp 再抓\n";
        if (!move_box::move_arms_to_belt_ready(*g_hw->arm_r, *g_hw->arm_l, c))
        {
            std::cerr << "[orch] " << name << " 手臂到准备姿态失败\n";
            return aborted() ? -4 : -5;
        }
        if (aborted())
            return -4;
    }

    const auto &off_l = c.grasp_offset_left;
    const auto &off_r = c.grasp_offset_right;
    std::cout << std::fixed << std::setprecision(4)
              << "[orch] belt_grasp 不转腰拍照，抓取点按码坐标系，左右可过中轴\n"
              << "[orch] belt_grasp 码原点=(" << det.origin_x << "," << det.origin_y << ","
              << det.origin_z << ") 左抓=(" << det.grasp_lx << "," << det.grasp_ly << ","
              << det.grasp_lz << ") offset_左=(" << off_l.x << "," << off_l.y << "," << off_l.z
              << ") 右抓=(" << det.grasp_rx << "," << det.grasp_ry << "," << det.grasp_rz
              << ") offset_右=(" << off_r.x << "," << off_r.y << "," << off_r.z
              << ") hover=" << c.grasp_hover_above_m
              << " rpy_deg 左=(" << c.grasp_rpy_left.rx << "," << c.grasp_rpy_left.ry << ","
              << c.grasp_rpy_left.rz << ") 右=(" << c.grasp_rpy_right.rx << ","
              << c.grasp_rpy_right.ry << "," << c.grasp_rpy_right.rz << ")\n";

    if (g_hw->g != nullptr)
    {
        double grip_r = c.grasp_ready_close_ratio;
        if (grip_r < 0.0)
            grip_r = 0.0;
        if (grip_r > 1.0)
            grip_r = 1.0;
        std::cout << std::fixed << std::setprecision(2)
                  << "[orch] belt_grasp 准备姿态夹爪收到 " << grip_r
                  << "（0全开 1全合），避免磕皮带边\n";
        log_gripper_qs("belt_grasp准备夹爪前");
        if (hand != 1 && !g_hw->last_grasped_l)
            g_hw->g->closeAndWait(
                gripper::Side::Left, grip_r, 0.15, std::chrono::milliseconds(3000));
        else if (g_hw->last_grasped_l)
            std::cout << "[orch] belt_grasp 左手已夹住，准备夹爪不动左手\n";
        if (hand != 0 && !right_arm_motors_locked() && !g_hw->last_grasped_r)
            g_hw->g->closeAndWait(
                gripper::Side::Right, grip_r, 0.15, std::chrono::milliseconds(3000));
        else if (g_hw->last_grasped_r)
            std::cout << "[orch] belt_grasp 右手已夹住，准备夹爪不动右手\n";
        log_gripper_qs("belt_grasp准备夹爪后");
        if (aborted())
            return -4;
    }

    const char *back_name_default = "固定关节中转准备位";
    // Empty grasp returns to the measured pre-approach configuration for a retry.
    wrist_servo::Joints retry_ready[2];
    const bool retry_ready_ok[2]={wrist_servo::read_joint_feedback(0,retry_ready[0]),
                                  right_arm_motors_locked() || wrist_servo::read_joint_feedback(1,retry_ready[1])};
    const int grasp_hand_sel = hand;
    auto depart_if_last = [&](bool is_right) {
        if (depart_after_right.empty() || !(is_right ? g_hw->last_grasped_r : g_hw->last_grasped_l))
            return;
        // 双手等待两侧收手完成；单手在本侧收手完成后离站。
        if (grasp_hand_sel < 0 && is_right)
            return;
        start_belt_depart_after_right(depart_after_right, is_right ? "右手" : "左手");
    };
    auto grasp_one = [&](bool is_right) -> int {
        if(is_right && right_arm_motors_locked()) return 0;
        if(!g_hw->g) return -1;
        t170::skills::ConveyorContext context{*g_hw->g,*g_hw->arm_r,*g_hw->arm_l,c,{}};
        t170::skills::ConveyorGraspRequest request;
        request.hand=is_right?t170::skills::Hand::Right:t170::skills::Hand::Left;
        request.metadata.workcell_id=station;request.log_tag=name;
        request.goal=move_box::make_belt_grasp_goal(is_right,
            is_right?det.grasp_rx:det.grasp_lx,is_right?det.grasp_ry:det.grasp_ly,
            is_right?det.grasp_rz:det.grasp_lz,c);
        const auto result=t170::skills::execute_conveyor_grasp(context,request,{aborted,{}});
        std::cout<<"[skill_result] "<<t170::skills::to_json(result)<<'\n';
        const auto &hand=result.hands[is_right?1:0];
        if(hand.action_completed) (is_right?g_hw->last_grasped_r:g_hw->last_grasped_l)=true;
        return skill_legacy_code(result);
    };

    // to_ready：空抓的手回到伸手前的准备 tcp，外层从这里直接重拍重抓。
    auto grasp_lift_and_back = [&](bool is_right,bool to_ready=false,
        const std::function<void(double)> &progress={},const std::function<void()> &on_return={}) -> int {
        if(is_right && right_arm_motors_locked()) return 0;
        if(!g_hw->g) return -1;
        t170::skills::ConveyorContext context{*g_hw->g,*g_hw->arm_r,*g_hw->arm_l,c,{}};
        t170::skills::ConveyorRetreatRequest request;
        request.hand=is_right?t170::skills::Hand::Right:t170::skills::Hand::Left;
        request.metadata.workcell_id=station;request.log_tag=name;
        request.retry_empty=to_ready;
        if(retry_ready_ok[is_right?1:0]) request.retry_ready=retry_ready[is_right?1:0];
        request.initial_part=to_ready?t170::skills::PartState::Empty:t170::skills::PartState::Holding;
        context.hooks.on_measured_lift=progress;context.hooks.on_return_started=on_return;
        const auto result=t170::skills::execute_conveyor_retreat(context,request,{aborted,{}});
        std::cout<<"[skill_result] "<<t170::skills::to_json(result)<<'\n';
        return skill_legacy_code(result);
    };

    auto report_picked = [&](bool is_right) -> int {
        if (!on_grasped)
            return 0;
        if (is_right && right_arm_motors_locked())
            return 0;
        return on_grasped(is_right);
    };

    if (hand == 0 || hand == 1)
    {
        const bool is_right = hand == 1;
        const int rc = grasp_one(is_right);
        if (rc == kBeltGraspEmpty)
        {
            const int back_rc = grasp_lift_and_back(is_right, true);
            if (back_rc != 0)
                return back_rc;
            empty_mask = is_right ? kEmptyRight : kEmptyLeft;
            return 0;
        }
        if (rc != 0)
            return rc;
        std::thread lift_th;
        const bool run_cb = !(is_right && right_arm_motors_locked());
        // 上抬一开始就回调（调度用来申请下一只手），与抬升/收手并行。抓完信号等这只手收回再发。
        if (on_lift && run_cb)
            lift_th = std::thread([&]() { on_lift(is_right); });
        const int back_rc = grasp_lift_and_back(is_right);
        if (lift_th.joinable())
            lift_th.join();
        if (back_rc != 0)
            return back_rc;
        const int note_rc = report_picked(is_right);
        if (note_rc != 0)
            return note_rc;
        depart_if_last(is_right);
        if (is_right &&
            station_name_matches(depart_after_right, g_move_cfg.chassis.tray2_station))
            start_tray2_waist_after_right_arrived();
        const int dep_rc = take_belt_depart();
        const int waist_rc = take_tray2_waist();
        if (dep_rc != 0)
            return dep_rc;
        if (waist_rc != 0)
            return waist_rc;
        std::cout << "[orch] " << name << (is_right ? " 右手" : " 左手")
                  << "单手抓取结束，停在 " << back_name_default << "\n";
        return 0;
    }

    // 双手：统一先右手，再左手（转运/下料两条传送带一致）。
    if (right_arm_motors_locked())
    {
        std::cout << "[orch] " << name << " 右臂锁定，双手模式退化为只抓左手\n";
        hand = 0;
        const int rc = grasp_one(false);
        if (rc == kBeltGraspEmpty)
        {
            const int back_rc = grasp_lift_and_back(false, true);
            if (back_rc != 0)
                return back_rc;
            empty_mask = kEmptyLeft;
            return 0;
        }
        if (rc != 0)
            return rc;
        const int back_rc = grasp_lift_and_back(false);
        if (back_rc != 0)
            return back_rc;
        const int note_rc = report_picked(false);
        if (note_rc != 0)
            return note_rc;
        depart_if_last(false);
        return 0;
    }

    int right_back_rc = 0;
    const int right_rc = grasp_one(true);
    if (right_rc == kBeltGraspEmpty)
    {
        std::cout << "[orch] " << name << " 右手空抓，左手这一轮不伸，回准备位再拍\n";
        const int back_rc = grasp_lift_and_back(true, true);
        if (back_rc != 0)
            return back_rc;
        empty_mask = kEmptyRight;
        return 0;
    }
    if (right_rc != 0)
        return right_rc;

    wrist_servo::RetreatClearanceGate right_clearance(c.next_hand_clearance_m);
    std::thread th_r([&]() {
        try { right_back_rc=grasp_lift_and_back(true,false,{},[&]{right_clearance.release();}); }
        catch(const std::exception &ex) {
            std::cerr<<"[orch] 右手收手异常: "<<ex.what()<<'\n';right_back_rc=-32;
        }
        right_clearance.finish(right_back_rc);
    });
    struct JoinRetreat {
        std::thread &worker;
        ~JoinRetreat() {if(worker.joinable()) worker.join();}
    } join_retreat{th_r};
    wrist_servo::task_event(1,"peer_wait_clearance","right_return_must_start_first");
    const int clearance_rc=right_clearance.wait(c.next_hand_clearance_timeout_s);
    if(clearance_rc!=0 || aborted()) {
        std::cerr<<"[orch] 右手未进入回程，左手禁止进入 code="<<clearance_rc<<'\n';
        return aborted()?-4:clearance_rc;
    }
    wrist_servo::task_event(1,"peer_clearance","right_return_started");
    std::cout << "[orch] belt_grasp 右手已在回程路上，左手进入\n";
    const int left_rc = grasp_one(false);
    const bool left_empty = left_rc == kBeltGraspEmpty;
    const int left_back_rc =
        (left_rc == 0 || left_empty) ? grasp_lift_and_back(false, left_empty) : 0;
    if (left_rc == 0 || left_empty)
    {
        if (left_back_rc != 0)
        {
            th_r.join();
            return left_back_rc;
        }
    }
    th_r.join();
    if (left_rc != 0 && !left_empty)
        return left_rc;
    if (left_back_rc != 0)
        return left_back_rc;
    if (right_back_rc != 0)
        return right_back_rc;
    if (left_empty)
        empty_mask = kEmptyLeft;
    if(!left_empty) {
        const int note_rc=report_picked(false);
        if(note_rc!=0) return note_rc;
        depart_if_last(false);
    }
    if(station_name_matches(depart_after_right,g_move_cfg.chassis.tray2_station) && !left_empty)
        start_tray2_waist_after_right_arrived();

    const int dep_rc = take_belt_depart();
    const int waist_rc = take_tray2_waist();
    if (dep_rc != 0)
        return dep_rc;
    if (waist_rc != 0)
        return waist_rc;
    if (empty_mask == 0)
        std::cout << "[orch] " << name << " 双手抓取结束（先右后左），停在 " << back_name_default
                  << "，可接入底盘/放置\n";
    return 0;
}

static int belt_recheck_after_empty(const ConveyorStationConfig &c, const char *tag)
{
    const char *name = (tag && tag[0]) ? tag : "belt_grasp";
    if (!c.yolo_roi_enable)
        return 0;
    if (g_hw == nullptr || g_hw->pipeline == nullptr)
    {
        if (g_hw != nullptr)
            g_hw->last_belt_message = "无相机";
        return kVisionQrFail;
    }
    SegPoseBridge &bridge = g_hw->pipeline->bridge();
    if (!bridge.belt_engine_ready())
    {
        g_hw->last_belt_message = "传送带引擎未加载";
        return kVisionQrFail;
    }
    std::cout << "[orch] " << name << " 空抓后再拍，看窗口里是倒件、立件还是没有\n";
    const int settle_ms = static_cast<int>(
        std::lround(std::max(0.0, c.grasp_photo_settle_sec) * 1000.0));
    if (settle_ms > 0)
        hardware_abort_sleep_ms(settle_ms);
    if (hardware_abort_requested())
        return -4;
    const double grasp_off_l[3] = {
        c.grasp_offset_left.x, c.grasp_offset_left.y, c.grasp_offset_left.z};
    const double grasp_off_r[3] = {
        c.grasp_offset_right.x, c.grasp_offset_right.y, c.grasp_offset_right.z};
    HeadBeltPerception perc = perceive_belt_head(
        g_hw->pipeline->cameras(), bridge, c.prefer_belt, kDebugVisualize, grasp_off_l,
        grasp_off_r, true);
    if (!perc.frame.ok)
    {
        g_hw->last_belt_message = "取帧失败";
        std::cerr << "[orch] " << name << " 空抓后再拍取帧失败: " << perc.frame.message << "\n";
        return kVisionQrFail;
    }
    if (!perc.belt.ok)
    {
        g_hw->last_belt_message = "二维码解算失败";
        std::cerr << "[orch] " << name << " 空抓后再拍二维码失败\n";
        return kVisionQrFail;
    }
    apply_fixed_belt_origin_z(perc.belt, c, name);
    const BeltPartList parts = detect_belt_parts_full(
        bridge, perc.frame, c, perc.belt, name, perc.yolo_ran ? &perc.yolo : nullptr);
    if (!parts.ok)
    {
        g_hw->last_belt_message = parts.message;
        std::cerr << "[orch] " << name << " 空抓后再拍检测失败: " << parts.message << "\n";
        return kVisionQrFail;
    }
    std::cout << "[orch] " << name << " 二次拍照结果：符合要求的检测框 "
              << parts.parts.size() << " 个\n";
    if (parts.parts.empty())
        std::cout << "[orch] " << name << " 二次拍照没有检测到符合要求的框\n";
    bool fallen = false;
    bool standing = false;
    for (int i = 0; i < static_cast<int>(parts.parts.size()); ++i)
    {
        const BeltPartMark &part = parts.parts[static_cast<size_t>(i)];
        log_belt_used_box(name, "二次拍照", i, part);
        if (!belt_part_in_grasp_window(part))
            continue;
        if (part.class_id == kFallenClassId)
            fallen = true;
        else
            standing = true;
    }
    if (fallen)
    {
        std::cout << "[orch] " << name << " 二次拍照判定：倒件\n";
        return 2;
    }
    if (standing)
    {
        std::cout << "[orch] " << name << " 二次拍照判定：立件\n";
        return 1;
    }
    std::cout << "[orch] " << name << " 二次拍照判定：抓取窗口内没有零件\n";
    return 0;
}

int go_belt_grasp(
    bool from_place, const ConveyorStationConfig &c, const std::string &station, const char *tag,
    int only_hand, const std::function<int(bool is_right)> &on_grasped,
    bool photo_settle, const std::string &depart_after_right,
    const std::function<void(bool is_right)> &on_lift,
    bool no_empty_retry)
{
    (void)no_empty_retry;
    const char *name = (tag && tag[0]) ? tag : "belt_grasp";
    int hand = only_hand;
    bool stood_retry = false;
    for (int attempt = 0;; ++attempt)
    {
        int empty_mask = 0;
        const int rc = go_belt_grasp_attempt(
            attempt == 0 ? from_place : true, c, station, tag, hand, on_grasped,
            attempt == 0 ? photo_settle : true, depart_after_right, on_lift, empty_mask);
        timing_event("grasp_attempt",name,rc==0 && empty_mask==0?"success":"failed",station,
                     {{"attempt",double(attempt+1)},{"code",double(rc)},{"empty_mask",double(empty_mask)}});
        if (rc != 0)
        {
            g_tray2_place_prep_done = false;
            return rc;
        }
        if (empty_mask == 0)
            return rc;
        const std::string who = empty_mask == kEmptyRight ? "右手" : "左手";
        const int kind = belt_recheck_after_empty(c, tag);
        if (kind < 0 || kind == kVisionQrFail)
        {
            g_tray2_place_prep_done = false;
            return kind;
        }
        if (kind == 2)
        {
            std::cerr << "[orch] " << name << " " << who << "空抓后再拍，窗口内是倒件\n";
            board_error(std::string(name) + " 空抓后再拍看到倒件");
            g_tray2_place_prep_done = false;
            return kBeltFallen;
        }
        if (kind == 1 && !stood_retry)
        {
            stood_retry = true;
            timing_event("retry",name,"retrying","empty grasp; standing part still present",{{"attempt",double(attempt+2)}});
            board_note("warn", std::string(name) + " " + who +
                                   "空抓后再拍仍是立件，在抓取窗口内再抓一次");
            if (only_hand < 0 && empty_mask == kEmptyLeft)
                hand = 0;
            if (only_hand < 0 && empty_mask == kEmptyRight)
                hand = -1;
            if (hardware_abort_requested())
            {
                g_tray2_place_prep_done = false;
                return -4;
            }
            continue;
        }
        std::cout << "[orch] " << name << " " << who
                  << (kind == 1 ? "再抓仍空，窗口里还是立件，不再抓\n"
                                : "空抓后再拍窗口内没有零件，按没抓到继续\n");
        g_tray2_place_prep_done = false;
        return kBeltGraspEmpty;
    }
}

int go_belt_grasp(bool from_place)
{
    return go_belt_grasp(
        from_place, g_move_cfg.conveyor, g_move_cfg.chassis.belt_station, "belt_grasp");
}
}
