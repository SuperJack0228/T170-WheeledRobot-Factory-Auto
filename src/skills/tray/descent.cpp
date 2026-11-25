#include "../internal/execution.h"
#include "t170/vision/tray_localizer.h"
#include "../internal/legacy_helpers.h"
#include "t170/motion/servo.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <numeric>
#include <optional>
#include <random>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "t170/vision/wrist_observer.h"
#include "t170/motion/arm.h"
#include "t170/common/units.h"
#include "t170/motion/feedback.h"
#include "t170/motion/kinematics_adapter.h"
#include "t170/motion/planning.h"
#include "t170/motion/servo_handoff.h"
#include <array>
#include <cstring>
#include "t170/vision/head_transform.h"
#include "t170/hardware/device_bus.h"

using std::vector;
using std::string;
using std::thread;


namespace move_box {
ArmLineMoveResult run_hand_approach_and_grasp(
        gripper::Gripper &g,
        Arm &arm_r,
        Arm &arm_l,
        HeadAssignState &st,
        HandMoveState &hs)
    {
        t170::skills::detail::report(t170::skills::Stage::AlignDescend,hs.move_r,hs.move_l);
        if(t170::skills::detail::active_execution && t170::skills::detail::active_execution->cancelled()) return {hs.move_r?-4:0,hs.move_l?-4:0};
        const bool straight_descent = true;
        const bool place=g_tray_hole_task==TrayHoleTask::PlaceEmpty;
        if(hs.move_r) st.released_r=false;
        if(hs.move_l) st.released_l=false;
        st.release_failed=false;
        wrist_servo::stage_tray_target(0,{});
        wrist_servo::stage_tray_target(1,{});
        if(hs.move_r) wrist_servo::invalidate_result(1);
        if(hs.move_l) wrist_servo::invalidate_result(0);
        log_phase_banner(straight_descent
                             ? "抓取流程：直线到孔正上方，再竖直下降"
                             : "抓取流程：从上方下压夹取");
        drop_if_y_split_blocks(st, hs, "hover/final下发前");
        if (hs.move_r && hs.move_l && dual_grasp_targets_too_close(st))
        {
            std::cout << "[col] 下压前列间隔不足，本轮只保留右手\n";
            hs.move_l = false;
        }

        // 改最终高度之前，goal_last 就是直线流程的物体上方点 B。
        const Eigen::Matrix<double, 1, 6> hover_r = st.goal_last_r;
        const Eigen::Matrix<double, 1, 6> hover_l = st.goal_last_l;

        if (hs.move_r &&
            !apply_final_grasp_z_from_hand(
                "右", true, st.row_r, hs.have_object_z_r, hs.object_z_r, st.goal_last_r))
        {
            hs.move_r = false;
        }
        if (hs.move_l &&
            !apply_final_grasp_z_from_hand(
                "左", false, st.row_l, hs.have_object_z_l, hs.object_z_l, st.goal_last_l))
        {
            hs.move_l = false;
        }
        if (!hs.move_r && !hs.move_l)
            return {};

        drop_if_y_split_blocks(st, hs, "final下发前");
        if (!hs.move_r && !hs.move_l)
            return {};

        // 自适应姿态在直线规划前确定，下降过程中不再改写终点。
        apply_adaptive_grasp_rpy_selective(arm_r, arm_l, st, hs.move_r, hs.move_l);

        Eigen::Matrix<double, 1, 6> hover_r_arm = arm_goal_in_real_waist_frame(hover_r);
        Eigen::Matrix<double, 1, 6> hover_l_arm = arm_goal_in_real_waist_frame(hover_l);
        Eigen::Matrix<double, 1, 6> goal_r_arm = arm_goal_in_real_waist_frame(st.goal_last_r);
        Eigen::Matrix<double, 1, 6> goal_l_arm = arm_goal_in_real_waist_frame(st.goal_last_l);

        struct ClearTrayGraspScope {
            ~ClearTrayGraspScope() {
                wrist_servo::stage_tray_grasp(0,std::numeric_limits<double>::quiet_NaN());
                wrist_servo::stage_tray_grasp(1,std::numeric_limits<double>::quiet_NaN());
            }
        } clear_tray_grasp_scope;
        for(int hand=0;hand<2;++hand) {
            const bool selected=hand?hs.move_r:hs.move_l;
            double feature_z=std::numeric_limits<double>::quiet_NaN();
            if(!place && selected && wrist_servo::calibrated_tray_grasp_for(g_move_cfg.visual_servo,hand)) {
                auto feature=hand?st.goal_last_r:st.goal_last_l;
                feature(2)=expected_object_top_z();
                feature_z=arm_goal_in_real_waist_frame(feature)(2);
            }
            wrist_servo::stage_tray_grasp(hand,feature_z);
        }

        if(place && straight_descent && g_move_cfg.cameras.hand_descent_correct &&
           g_move_cfg.visual_servo.tray_place.enabled) {
            wrist_servo::TrayBoardGeometry board; std::string error;
            const auto path=std::filesystem::path(g_move_cfg.visual_servo.tray_place.board_config);
            const auto absolute=path.is_absolute() ? path : std::filesystem::path(project_root_dir())/path;
            if(!wrist_servo::load_tray_board(absolute.string(),board,error)) {
                std::cerr << "[tray_place] board geometry: " << error << std::endl;
                return {hs.move_r?-39:0,hs.move_l?-39:0};
            }
            auto prepare=[&](int hand,int row,int col) {
                wrist_servo::TrayTarget target;
                auto fixed_plane_anchors=st.tray_anchors;
                for(auto &anchor:fixed_plane_anchors) anchor.base.z()=tray_z_ref_now();
                if(!wrist_servo::make_tray_target(fixed_plane_anchors,board,row,col,target,error)) return false;
                // The factory tray is horizontal. Keep head XY/yaw for identity,
                // but use the calibrated station plane instead of head vision Z/tilt.
                const double yaw=std::atan2(target.base_from_tray(1,0),target.base_from_tray(0,0));
                target.base_from_tray.block<3,3>(0,0)=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
                target.base_from_tray(2,3)=tray_z_ref_now();
                const Eigen::Matrix3d r=rot_y(g_waist_pitch_rad)*rot_x(g_waist_roll_rad)*rot_z(g_waist_yaw_rad);
                Eigen::Matrix4d current_from_upright=Eigen::Matrix4d::Identity();
                current_from_upright.block<3,3>(0,0)=r.transpose();
                current_from_upright.block<3,1>(0,3)=-r.transpose()*Eigen::Vector3d(
                    kWaistPitchLinkM*std::sin(g_waist_pitch_rad),0,
                    kWaistPitchLinkM*(std::cos(g_waist_pitch_rad)-1));
                target.base_from_tray=current_from_upright*target.base_from_tray;
                wrist_servo::stage_tray_target(hand,target);
                std::cout << "[tray_place] hand=" << hand << " fixed row=" << row << " col=" << col
                          << " RGB affine-neighbor B-observation; fixed plane_z="
                          <<target.base_from_tray(2,3)<<" no head depth/normal\n";
                return true;
            };
            if((hs.move_r && !prepare(1,st.row_aruco_r,st.col_aruco_r)) ||
               (hs.move_l && !prepare(0,st.row_aruco_l,st.col_aruco_l))) {
                std::cerr << "[tray_place] cannot bind assigned hole: " << error << std::endl;
                return {hs.move_r?-39:0,hs.move_l?-39:0};
            }
        }

        if(g_move_cfg.visual_servo.require_dual &&
           (!hs.move_r || !hs.move_l || !head_grasp_simultaneous(st))) {
            std::cerr << "[servo] 双手验证需要两侧有效目标且满足原有列间距/避让约束；本次不降级为单手\n";
            return {-34,-34};
        }
        // 料盘抓取：hand_descent_correct 即可进伺服。
        // Both tray routes require wrist correction; disabled configuration is rejected.
        const bool use_visual_descent =
            g_move_cfg.cameras.hand_descent_correct &&
            (!place || g_move_cfg.visual_servo.tray_place.enabled);
        if (!use_visual_descent) {
            std::cerr << "[tray] Open-loop tray fallback retired: enable wrist correction before grasp/place\n";
            return {hs.move_r ? -30 : 0, hs.move_l ? -30 : 0};
        }
        ArmLineMoveResult out;
        if (straight_descent)
        {
            ArmLineMoveDebugStage stage("A→B→C 直线下降");
            const double approach_mps = g_move_cfg.head_grasp.approach_vel_m_s;
            double kDescentMps = g_move_cfg.head_grasp.descend_vel_m_s;
            if (!std::isfinite(kDescentMps) || kDescentMps <= 0)
                return {hs.move_r ? -30 : 0, hs.move_l ? -30 : 0};
            const int hand_class = g_tray_hole_task == TrayHoleTask::PlaceEmpty ? 1 : 0;
            std::cout << std::fixed << std::setprecision(2)
                      << "[descent] 直线 " << approach_mps
                      << " m/s 接近孔正上方，随后竖直 " << kDescentMps
                      << " m/s 下降。";
            if (use_visual_descent)
                std::cout << "下降段使用双轴闭环、延迟补偿与预测制动\n";
            out = arm_dual_line_then_line_nostop(
                arm_r, hover_r_arm, goal_r_arm, hs.move_r,
                arm_l, hover_l_arm, goal_l_arm, hs.move_l,
                approach_mps, kDescentMps, {}, false, place ? "料盘放置下降" : "下降", false,
                use_visual_descent, hand_class);
        }

        if (hs.move_r && out.ret_r != 0)
        {
            log_arm_traj_plan_fail("右手", out.ret_r);
            return out;
        }
        if (hs.move_l && out.ret_l != 0)
        {
            log_arm_traj_plan_fail("左手", out.ret_l);
            return out;
        }

        // Servo owns stable completion only when this path actually ran visual descent.
        const bool servo_descent = straight_descent && use_visual_descent;
        if(servo_descent && !place)
            return finish_servo_grasp(g,arm_r,arm_l,hs.move_r,hs.move_l,
                                      st.empty_grasp_r,st.empty_grasp_l,"料盘抓取");
        // Keep the last joint reference streaming until retract playback takes it.
        // Stopping it when the claw feedback returns leaves a gap, then the retract
        // trajectory's first sample is not that joint pose and the arm yanks.
        if (servo_descent && ((hs.move_r && out.ret_r == 0) || (hs.move_l && out.ret_l == 0)))
            wrist_servo::start_final_hold_repeat();
        if(servo_descent) {
            if(hs.move_r) wrist_servo::task_event(1,"both_ready","checking_before_gripper");
            if(hs.move_l) wrist_servo::task_event(0,"both_ready","checking_before_gripper");
            const bool right_ready=!hs.move_r || wrist_servo::ready_for_gripper(arm_r);
            const bool left_ready=!hs.move_l || wrist_servo::ready_for_gripper(arm_l);
            if(!right_ready || !left_ready) {
                std::cerr << "[servo] " << (place ? "张爪" : "合爪") << "前位置复核失败，详见 gripper_gate 日志\n";
                return {hs.move_r?-36:0,hs.move_l?-36:0};
            }
        } else {
            if (!wait_arms_motors_stopped(hs.move_r, hs.move_l, 5.0, 6, "最终C到位"))
            {
                std::cerr << "[grasp] 最终C电机到位等待超时，禁止合爪\n";
                if (hs.move_r)
                    out.ret_r = -8;
                if (hs.move_l)
                    out.ret_l = -8;
                return out;
            }

            // 位置偏差以最终抓取点为基准。
            auto verify_final_c = [&](Arm &arm,
                                      const Eigen::Matrix<double, 1, 6> &goal,
                                      int hand_id,
                                      const char *hand,
                                      int &ret_io) {
                const auto &hg = g_move_cfg.head_grasp;
                Eigen::Matrix<double, 1, 6> expect = goal;
                double vis_x = 0.0;
                double vis_y = 0.0;
                const bool visual_xy = descent_commanded_xy(hand_id, vis_x, vis_y);
                if (visual_xy)
                {
                    expect(0) = vis_x;
                    expect(1) = vis_y;
                }
                Eigen::Matrix<double, 1, 6> cur = arm_get_tcp_pos(arm);
                double xyz_err = 0.0;
                double rpy_err = 0.0;
                const double xyz_tol = std::max(0.001, hg.bezier_endpoint_xyz_tol_m);
                const double rpy_tol = std::max(0.5, hg.bezier_endpoint_rpy_tol_deg) * rad;
                bool ok = false;
                int stable_samples = 0;
                // 编码器停稳后复核 1 次即可；超差最多再看 0.3s。
                for (int sample = 0; sample < 6; ++sample)
                {
                    cur = arm_get_tcp_pos(arm);
                    xyz_err = (cur.head<3>() - expect.head<3>()).norm();
                    rpy_err = 0.0;
                    for (int axis = 3; axis < 6; ++axis)
                    {
                        const double d = std::atan2(
                            std::sin(cur(axis) - expect(axis)),
                            std::cos(cur(axis) - expect(axis)));
                        rpy_err = std::max(rpy_err, std::abs(d));
                    }
                    const bool sample_ok = xyz_err <= xyz_tol && rpy_err <= rpy_tol;
                    stable_samples = sample_ok ? stable_samples + 1 : 0;
                    if (stable_samples >= 1)
                    {
                        ok = true;
                        break;
                    }
                    if (hardware_abort_requested())
                        break;
                    hardware_abort_sleep_ms(50);
                }
                std::cout << std::fixed << std::setprecision(4)
                          << "[到位] " << hand
                          << (visual_xy ? " 按图像修正点" : " 按头相机目标")
                          << " target=(" << expect(0) << ',' << expect(1) << ',' << expect(2) << ')'
                          << " actual=(" << cur(0) << ',' << cur(1) << ',' << cur(2) << ')'
                          << " xyz_err=" << xyz_err << "m rpy_err=" << (rpy_err / rad) << "deg\n";
                if (!ok)
                {
                    std::cerr << "[到位] " << hand << " 最终点超差，禁止"
                              << (g_tray_hole_task == TrayHoleTask::PlaceEmpty ? "张爪" : "合爪")
                              << "；请勿把轨迹误差混入视觉offset\n";
                    ret_io = -7;
                }
            };
            if (hs.move_r)
                verify_final_c(arm_r, goal_r_arm, 1, "右", out.ret_r);
            if (hs.move_l)
                verify_final_c(arm_l, goal_l_arm, 0, "左", out.ret_l);

        }

        const int settle_ms = static_cast<int>(
            std::max(0.0, place ? g_move_cfg.tray2_place.pre_open_settle_sec : g_move_cfg.head_grasp.pre_grasp_settle_sec) * 1000.0);
        if (!servo_descent && settle_ms > 0)
            hardware_abort_sleep_ms(settle_ms);
        if (hardware_abort_requested())
        {
            out.ret_r = hs.move_r ? -4 : out.ret_r;
            out.ret_l = hs.move_l ? -4 : out.ret_l;
            return out;
        }

        t170::skills::detail::report(t170::skills::Stage::Gripper,hs.move_r,hs.move_l);
        if(place) {
            bool released[2]={false,false};
            const auto &p=g_move_cfg.tray2_place;
            auto open=[&](int hand) {
                hand_shadow_end(hand);
                wrist_servo::task_event(hand,"gripper_start","open");
                try {
                    const auto side=hand==1 ? gripper::Side::Right : gripper::Side::Left;
                    // The SDK treats an unpaired side as success; placement must
                    // confirm real feedback before clearing a held-part latch.
                    if(g.hasSide(side)) {
                        const double open_q=g.config().open_position_rad;
                        const double close_q=g.config().close_position_rad;
                        const double target=std::clamp(p.release_q_rad,std::min(open_q,close_q),std::max(open_q,close_q));
                        const double span=close_q-open_q;
                        const double ratio=std::abs(span)<1e-6?0.0:std::clamp((target-open_q)/span,0.0,1.0);
                        const bool waited=g.closeAndWait(side,ratio,p.open_tolerance_rad,std::chrono::milliseconds(p.open_timeout_ms));
                        const auto fb=g.feedback(side);
                        released[hand]=waited && fb.have_feedback && fb.io_ok && fb.merror==0 &&
                            std::abs(fb.position_rad-target)<=p.open_tolerance_rad;
                        wrist_servo::record_gripper_check(hand,fb.position_rad,fb.torque,
                                                         released[hand] ? "open_confirmed" : "open_not_confirmed");
                    }
                }
                catch(const std::exception &ex) { std::cerr << "[tray_place] open exception: " << ex.what() << std::endl; }
                wrist_servo::task_event(hand,"gripper_end",released[hand] ? "open_confirmed" : "open_failed");
            };
            std::thread right,left;
            if(hs.move_r && out.ret_r==0) right=std::thread([&]{open(1);});
            if(hs.move_l && out.ret_l==0) left=std::thread([&]{open(0);});
            if(right.joinable()) right.join();
            if(left.joinable()) left.join();
            if(hs.move_r && out.ret_r==0) {
                st.released_r=released[1];
                t170::skills::detail::report_action(true,released[1]?t170::skills::PartState::Empty:t170::skills::PartState::Unknown,t170::skills::Evidence::GripperFeedback,released[1]);
                if(!released[1]) {out.ret_r=-38;st.release_failed=true;}
            }
            if(hs.move_l && out.ret_l==0) {
                st.released_l=released[0];
                t170::skills::detail::report_action(false,released[0]?t170::skills::PartState::Empty:t170::skills::PartState::Unknown,t170::skills::Evidence::GripperFeedback,released[0]);
                if(!released[0]) {out.ret_l=-38;st.release_failed=true;}
            }
            if(st.release_failed) {
                if(hs.move_r) arm_discard_line_then_line_plan(1);
                if(hs.move_l) arm_discard_line_then_line_plan(0);
            }
            return out;
        }

        return out;
    }
}
