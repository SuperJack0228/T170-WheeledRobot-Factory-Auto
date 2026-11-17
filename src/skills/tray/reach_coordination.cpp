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
#include "Ti5_Arm.h"
#include "t170/common/units.h"
#include "t170/motion/cartesian_tracking.h"
#include "t170/motion/retreat.h"
#include "t170/motion/servo_handoff.h"
#include <array>
#include <cstring>
#include "t170/vision/head_transform.h"
#include "t170/hardware/device_bus.h"

using std::vector;
using std::cout;
using std::thread;


namespace move_box {
double waist_x_arm_goal_comp(double waist_delta_x)
    {
        return -waist_delta_x;
    }

double waist_layer3_stagger_forward(
        WaistRobot &waist,
        Eigen::Matrix<double, 1, 6> &posup_down_3_layer,
        int multiplier,
        const char *log_tag)
    {
        if (multiplier <= 0)
            return 0.0;
        if (!waist_layer3_is_synced())
        {
            std::cerr << "[waist] layer3 未与编码器同步，拒绝腰进（先执行 home）\n";
            return 0.0;
        }
        const double cur_x = posup_down_3_layer(0);
        const double delta = static_cast<double>(multiplier) * g_move_cfg.waist.stagger_step_x;
        std::ostringstream oss;
        oss << log_tag << " x×" << multiplier << " step=" << g_move_cfg.waist.stagger_step_x
            << " delta=" << delta;
        posup_down_3_layer(0) += delta;
        clamp_waist_layer3_xyz(posup_down_3_layer);
        const double actual = posup_down_3_layer(0) - cur_x;
        if (actual <= 1e-9)
        {
            std::cout << oss.str() << "，已到 x 行程上限，跳过\n";
            return 0.0;
        }
        std::cout << oss.str() << "，实际前进=" << actual << " m\n";
        const int rc = waist.moveLToPos(posup_down_3_layer, 0.1);
        if (rc != 0)
        {
            std::cerr << "[waist] stagger 运动失败 code=" << rc << "\n";
            sync_waist_layer3_from_encoders(waist, posup_down_3_layer, "腰进失败后");
            return 0.0;
        }
        hardware_abort_sleep_ms(
            static_cast<int>(g_move_cfg.waist.move_settle_sec * 1000.0));
        read_arrived_pose(waist, posup_down_3_layer);
        g_current_waist_layer3_z = posup_down_3_layer(2);
        return posup_down_3_layer(0) - cur_x;
    }

double waist_layer3_ensure_far_row_forward(
        WaistRobot &waist,
        Eigen::Matrix<double, 1, 6> &layer3,
        double forward_m)
    {
        const double want = forward_m >= 0.0 ? forward_m : g_move_cfg.waist.far_row_forward_m;
        if (want <= 1e-9)
            return 0.0;
        if (!waist_layer3_is_synced())
        {
            std::cerr << "[waist] layer3 未与编码器同步，拒绝远三排腰进（先执行 home）\n";
            return 0.0;
        }
        if (!sync_waist_layer3_from_encoders(waist, layer3, "远三排腰进前"))
            return 0.0;
        const double x_fwd = g_move_cfg.waist.layer3_home(0) + want;
        const double cur_x = layer3(0);
        layer3(0) = x_fwd;
        clamp_waist_layer3_xyz(layer3);
        const double actual = layer3(0) - cur_x;
        if (actual <= 0.01)
        {
            std::cout << std::fixed << std::setprecision(4)
                      << "[waist] 远三排已在前伸位 x=" << cur_x
                      << " 目标=" << x_fwd << " m，跳过\n";
            layer3(0) = cur_x;
            return 0.0;
        }
        std::cout << std::fixed << std::setprecision(4)
                  << "[waist] 远三排腰进 x " << cur_x << " → " << layer3(0)
                  << " (home " << g_move_cfg.waist.layer3_home(0) << " 前伸 " << want
                  << " m，锁 y/z/姿态)\n";
        const int rc = waist.moveLToPos(layer3, 0.1);
        if (rc != 0)
        {
            std::cerr << "[waist] 远三排腰进失败 code=" << rc << "\n";
            sync_waist_layer3_from_encoders(waist, layer3, "远三排腰进失败后");
            return 0.0;
        }
        hardware_abort_sleep_ms(
            static_cast<int>(g_move_cfg.waist.move_settle_sec * 1000.0));
        read_arrived_pose(waist, layer3);
        const double moved = layer3(0) - cur_x;
        return moved > 1e-9 ? moved : 0.0;
    }

void apply_grasp_stagger_waist_comp(HeadAssignState &st, double waist_actual_delta)
    {
        if (std::abs(waist_actual_delta) <= 1e-9)
            return;
        const double comp = waist_x_arm_goal_comp(waist_actual_delta);
        for(auto &anchor:st.tray_anchors) anchor.base.x()+=comp;
        if (st.have_r)
            st.goal_last_r(0) += comp;
        if (st.have_l)
            st.goal_last_l(0) += comp;
        refresh_head_move_flags(st);
    }



bool restore_waist_layer3_x(WaistRobot &waist, Eigen::Matrix<double, 1, 6> &posup_down_3_layer)
    {
        if (!sync_waist_layer3_from_encoders(waist, posup_down_3_layer, "恢复x前") &&
            !waist_layer3_is_synced())
        {
            std::cerr << "[waist] layer3 未与编码器同步，拒绝恢复 x\n";
            return false;
        }
        const double home_x = g_move_cfg.waist.layer3_home(0);
        const double cur_x = posup_down_3_layer(0);
        posup_down_3_layer(0) = home_x;
        clamp_waist_layer3_xyz(posup_down_3_layer);
        if (std::abs(cur_x - posup_down_3_layer(0)) <= 0.01)
        {
            posup_down_3_layer(0) = cur_x;
            return true;
        }
        std::cout << std::fixed << std::setprecision(4)
                  << "[waist] 恢复 layer3 x " << cur_x << " → " << posup_down_3_layer(0)
                  << "（锁 y/z/姿态）\n";
        const int rc = waist.moveLToPos(posup_down_3_layer, 0.1);
        if (rc != 0)
        {
            std::cerr << "[waist] 恢复 layer3 x 失败 code=" << rc << "\n";
            sync_waist_layer3_from_encoders(waist, posup_down_3_layer, "恢复x失败后");
            return false;
        }
        hardware_abort_sleep_ms(static_cast<int>(g_move_cfg.waist.move_settle_sec * 1000.0));
        if (!sync_waist_layer3_from_encoders(waist, posup_down_3_layer, "恢复x后"))
            g_current_waist_layer3_z = posup_down_3_layer(2);
        return true;
    }

bool ensure_waist_layer3_x_home(WaistRobot &waist, Eigen::Matrix<double, 1, 6> &posup_down_3_layer)
    {
        if (!sync_waist_layer3_from_encoders(waist, posup_down_3_layer, "回腰前"))
            return false;
        const double home_x = g_move_cfg.waist.layer3_home(0);
        if (std::abs(posup_down_3_layer(0) - home_x) <= 0.01)
        {
            std::cout << std::fixed << std::setprecision(4)
                      << "[waist] 已在 home x=" << posup_down_3_layer(0) << " m，不回退\n";
            return true;
        }
        std::cout << std::fixed << std::setprecision(4)
                  << "[waist] 当前 x=" << posup_down_3_layer(0)
                  << " 不在 home " << home_x << "，先平移回去再升降\n";
        return restore_waist_layer3_x(waist, posup_down_3_layer);
    }

bool ensure_waist_ready_start(WaistRobot &waist, Eigen::Matrix<double, 1, 6> &posup_down_3_layer)
    {
        const auto &w = g_move_cfg.waist;
        const double x_ready = waist_ready_x();
        const double z_ready = waist_ready_z();
        std::cout << std::fixed << std::setprecision(4)
                  << "[waist] ready/grasp 起始 x=" << x_ready
                  << "（home_x=" << w.layer3_home(0) << " 再向前 "
                  << w.ready_forward_m << " m）z=" << z_ready
                  << "（home z=" << w.layer3_home(2) << "）\n";
        if (!waist_layer3_move_x_only(waist, posup_down_3_layer, x_ready, false, false))
            return false;
        if (!waist_layer3_move_z_only(waist, posup_down_3_layer, z_ready, false, false))
            return false;
        hardware_abort_sleep_ms(static_cast<int>(g_move_cfg.waist.move_settle_sec * 1000.0));
        read_arrived_pose(waist, posup_down_3_layer);
        std::cout << std::fixed << std::setprecision(4)
                  << "[waist] ready 已停，实际 x=" << posup_down_3_layer(0)
                  << " z=" << posup_down_3_layer(2)
                  << " 目标 x=" << x_ready << " z=" << z_ready << " m\n";
        return true;
    }

bool stagger_target_side_graspable(const HeadAssignState &st, bool for_right)
    {
        return for_right ? (st.have_r && st.move_r) : (st.have_l && st.move_l);
    }

bool stagger_target_side_x_strict_ok(const HeadAssignState &st, bool for_right)
    {
        if (!stagger_target_side_graspable(st, for_right))
            return false;
        const double x = for_right ? st.goal_last_r(0) : st.goal_last_l(0);
        return !head_x_needs_stagger(x);
    }

bool stagger_waist_budget_exhausted(const GraspStaggerWaistState &sw)
    {
        return sw.steps_used >= std::max(1, g_move_cfg.waist.stagger_max_steps);
    }

bool advance_waist_stagger_side(
        WaistRobot &waist,
        Eigen::Matrix<double, 1, 6> &posup_down_3_layer,
        bool for_right,
        HeadAssignState &st,
        GraspStaggerWaistState &stagger_waist)
    {
        const int max_steps = std::max(1, g_move_cfg.waist.stagger_max_steps);
        const char *hand = for_right ? "右手" : "左手";

        if (stagger_target_side_x_strict_ok(st, for_right))
            return true;
        if (!stagger_target_side_graspable(st, for_right))
            return false;

        if (stagger_waist_budget_exhausted(stagger_waist))
        {
            const double x = for_right ? st.goal_last_r(0) : st.goal_last_l(0);
            cout << "[stagger] " << hand << " 本轮腰进已用满 " << max_steps << " 步，x=" << x
                 << " 仍>" << g_move_cfg.stagger.head_x_threshold
                 << "，grasp_valid 合格，继续抓取\n";
            return true;
        }

        const int remaining = max_steps - stagger_waist.steps_used;
        const double x = for_right ? st.goal_last_r(0) : st.goal_last_l(0);
        const int mult = stagger_waist_steps_for_excess(x, remaining);
        if (mult <= 0)
            return true;

        cout << "[stagger] " << hand << " 目标x=" << x << " 一次性腰进×" << mult << " (本轮 "
             << stagger_waist.steps_used << "+" << mult << "/" << max_steps
             << "，头相机不重拍)\n";
        const double actual = waist_layer3_stagger_forward(
            waist, posup_down_3_layer, mult, "[waist] stagger抓取腰进");
        apply_grasp_stagger_waist_comp(st, actual);
        stagger_waist.steps_used += mult;

        if (stagger_target_side_x_strict_ok(st, for_right))
            return true;
        if (stagger_waist_budget_exhausted(stagger_waist))
        {
            cout << "[stagger] " << hand << " 腰进后本轮已达 " << max_steps
                 << " 步，grasp_valid 合格，继续抓取\n";
            return stagger_target_side_graspable(st, for_right);
        }
        return stagger_target_side_graspable(st, for_right);
    }

bool advance_waist_stagger_dual_hands(
        WaistRobot &waist,
        Eigen::Matrix<double, 1, 6> &posup_down_3_layer,
        HeadAssignState &st,
        GraspStaggerWaistState &stagger_waist)
    {
        const int max_steps = std::max(1, g_move_cfg.waist.stagger_max_steps);

        auto dual_x_strict_ok = [&]() -> bool
        {
            if (st.move_r && head_x_needs_stagger(st.goal_last_r(0)))
                return false;
            if (st.move_l && head_x_needs_stagger(st.goal_last_l(0)))
                return false;
            return st.move_r || st.move_l;
        };
        auto dual_graspable = [&]() -> bool { return st.move_r || st.move_l; };

        if (dual_x_strict_ok())
            return true;
        if (!dual_graspable())
            return false;

        if (stagger_waist_budget_exhausted(stagger_waist))
        {
            cout << "[stagger] 双手本轮腰进已用满 " << max_steps
                 << " 步，grasp_valid 合格，继续抓取\n";
            return true;
        }

        double target_x = 0.0;
        if (st.move_r && head_x_needs_stagger(st.goal_last_r(0)))
            target_x = std::max(target_x, st.goal_last_r(0));
        if (st.move_l && head_x_needs_stagger(st.goal_last_l(0)))
            target_x = std::max(target_x, st.goal_last_l(0));

        const int remaining = max_steps - stagger_waist.steps_used;
        const int mult = stagger_waist_steps_for_excess(target_x, remaining);
        if (mult <= 0)
            return true;

        cout << "[stagger] 双手最远x=" << target_x << " 一次性腰进×" << mult << " (本轮 "
             << stagger_waist.steps_used << "+" << mult << "/" << max_steps
             << "，头相机不重拍)\n";
        const double actual = waist_layer3_stagger_forward(
            waist, posup_down_3_layer, mult, "[waist] stagger双手腰进");
        apply_grasp_stagger_waist_comp(st, actual);
        stagger_waist.steps_used += mult;

        if (dual_x_strict_ok())
            return true;
        if (stagger_waist_budget_exhausted(stagger_waist))
        {
            cout << "[stagger] 双手腰进后本轮已达 " << max_steps << " 步，继续抓取\n";
            return dual_graspable();
        }
        return dual_graspable();
    }
}
