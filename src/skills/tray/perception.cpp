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
TrayZoneCount count_zone_from_tray(const TrayDetectResult &tray)
    {
        TrayZoneCount out;
        if (!tray.ok)
        {
            out.message = tray.message.empty() ? "料盘解算失败" : tray.message;
            return out;
        }

        const auto &valid = g_move_cfg.grasp_valid;
        const int right_max_col = std::max(1, g_move_cfg.grasp_zone.column_count / 2);
        const int want_class = tray_assign_class_id();
        std::map<int, std::pair<double, int>> col_y_acc;
        for (const TrayHoleResult &h : tray.holes)
        {
            const double y = std::isfinite(h.y_level) ? h.y_level : h.y;
            if (!h.in_robot || !std::isfinite(y) || h.col < 1)
                continue;
            col_y_acc[h.col].first += y;
            col_y_acc[h.col].second += 1;
        }
        bool cols_flipped = false;
        if (col_y_acc.size() >= 2)
        {
            const int c_lo = col_y_acc.begin()->first;
            const int c_hi = col_y_acc.rbegin()->first;
            if (c_hi > c_lo && col_y_acc[c_lo].second > 0 && col_y_acc[c_hi].second > 0)
            {
                const double y_lo = col_y_acc[c_lo].first / static_cast<double>(col_y_acc[c_lo].second);
                const double y_hi = col_y_acc[c_hi].first / static_cast<double>(col_y_acc[c_hi].second);
                cols_flipped = y_lo > y_hi;
            }
        }

        for (const TrayHoleResult &h : tray.holes)
        {
            if (h.class_id != want_class)
                continue;
            const double x = std::isfinite(h.x_level) ? h.x_level : h.x;
            const double y = std::isfinite(h.y_level) ? h.y_level : h.y;
            if (!h.in_robot || !std::isfinite(x) || !std::isfinite(y))
                continue;
            if (x < valid.x_min || x > valid.x_max)
                continue;
            const int col = tray_assign_col_from_aruco(h.col, cols_flipped);
            if (col < 1)
                continue;
            const bool is_right_col = col <= right_max_col;
            if (is_right_col
                    ? (y < valid.right_y_min || y > valid.right_y_max)
                    : (y < valid.left_y_min || y > valid.left_y_max))
                continue;
            if (is_right_col)
                ++out.right;
            else
                ++out.left;
        }
        out.ok = true;
        std::cout << "[tray] 工作区 "
                  << (want_class == kEmptyHoleClassId ? "空孔" : "立件")
                  << " 右=" << out.right << " 左=" << out.left
                  << (cols_flipped ? "（列号已对调）\n" : "\n");
        return out;
    }

TrayZoneCount count_tray_zone_holes(
        RealSenseMultiCam &cameras,
        SegPoseBridge &bridge,
        const std::array<double, 16> &cam2robot)
    {
        TrayZoneCount out;
        if (app_stop_requested() || hardware_abort_requested())
        {
            out.message = "已中止";
            return out;
        }
        if (!bridge.tray_engine_ready())
        {
            out.message = "料盘引擎未就绪";
            return out;
        }
        (void)cam2robot;
        PerceiveTrayOptions opts;
        opts.run_yolo = true;
        opts.tray2_markers = tray2_place_active();
        const HeadTrayPerception perc = perceive_tray_head(cameras, bridge, opts);
        return count_zone_from_tray(perc.tray);
    }

bool assign_from_tray_holes(
        const TrayDetectResult &tray,
        HeadAssignState &st,
        bool skip_right,
        bool skip_left)
    {
        auto set_invalid = [](double pos[3]) {
            pos[0] = -1.0;
            pos[1] = 0.0;
            pos[2] = 0.0;
        };
        set_invalid(st.right_hand_pos);
        set_invalid(st.left_hand_pos);
        st.xy_from_tray = false;
        st.row_r = 0;
        st.row_l = 0;
        st.row_aruco_r = 0;
        st.row_aruco_l = 0;
        st.col_r = 0;
        st.col_l = 0;
        st.col_aruco_r = 0;
        st.col_aruco_l = 0;
        st.zones = {};
        st.tray_anchors.clear();
        if (!tray.ok)
            return false;
        for(const auto &h:tray.holes)
            if(h.in_robot && std::isfinite(h.x) && std::isfinite(h.y) && std::isfinite(h.tray_z))
                st.tray_anchors.push_back({h.row,h.col,Eigen::Vector3d(h.x,h.y,h.tray_z),h.class_id});

        struct Cand
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            int id = 0;
            int col = 0;
            int row = 0;
            int row_aruco = 0;
            int col_aruco = 0;
            double conf = 0.0;
        };

        const auto &valid = g_move_cfg.grasp_valid;
        const int right_max_col = std::max(1, g_move_cfg.grasp_zone.column_count / 2);

        double origin_y = tray.origin_y;
        if (!std::isfinite(origin_y))
        {
            double acc = 0.0;
            int n = 0;
            for (const TrayHoleResult &h : tray.holes)
            {
                const double y = std::isfinite(h.y_level) ? h.y_level : h.y;
                if (!h.in_robot || !std::isfinite(y))
                    continue;
                acc += y;
                ++n;
            }
            if (n > 0)
                origin_y = acc / static_cast<double>(n);
        }
        if (std::isfinite(origin_y))
            set_live_tray_origin_y(origin_y);

        std::map<int, std::pair<double, int>> col_y_acc;
        for (const TrayHoleResult &h : tray.holes)
        {
            const double y = std::isfinite(h.y_level) ? h.y_level : h.y;
            if (!h.in_robot || !std::isfinite(y) || h.col < 1)
                continue;
            col_y_acc[h.col].first += y;
            col_y_acc[h.col].second += 1;
        }
        bool cols_flipped = false;
        if (col_y_acc.size() >= 2)
        {
            const int c_lo = col_y_acc.begin()->first;
            const int c_hi = col_y_acc.rbegin()->first;
            if (c_hi > c_lo && col_y_acc[c_lo].second > 0 && col_y_acc[c_hi].second > 0)
            {
                const double y_lo = col_y_acc[c_lo].first / static_cast<double>(col_y_acc[c_lo].second);
                const double y_hi = col_y_acc[c_hi].first / static_cast<double>(col_y_acc[c_hi].second);
                // ArUco col1 在盘系 -Y（机器人右侧）。若小号列的基座 Y 反而更大，说明盘转了 180°。
                cols_flipped = y_lo > y_hi;
            }
        }

        std::map<int, std::pair<double, int>> row_x_acc;
        for (const TrayHoleResult &h : tray.holes)
        {
            const double x = std::isfinite(h.x_level) ? h.x_level : h.x;
            if (!h.in_robot || !std::isfinite(x) || h.row < 1)
                continue;
            row_x_acc[h.row].first += x;
            row_x_acc[h.row].second += 1;
        }
        std::vector<std::pair<double, int>> ranked;
        ranked.reserve(row_x_acc.size());
        for (const auto &kv : row_x_acc)
        {
            if (kv.second.second <= 0)
                continue;
            ranked.push_back({kv.second.first / static_cast<double>(kv.second.second), kv.first});
        }
        std::sort(ranked.begin(), ranked.end(),
                  [](const std::pair<double, int> &a, const std::pair<double, int> &b) {
                      return a.first > b.first;
                  });
        std::map<int, int> tray_to_from_robot;
        for (size_t i = 0; i < ranked.size(); ++i)
            tray_to_from_robot[ranked[i].second] = static_cast<int>(i) + 1;

        const int want_class = tray_assign_class_id();
        std::cout << "\n--- 料盘孔位分配 (ArUco XY + YOLO class" << want_class
                  << (want_class == kEmptyHoleClassId
                          ? (g_tray_hole_task == TrayHoleTask::PlaceEmpty ? " 空孔放置" : " 空孔")
                          : "")
                  << ") ---\n";
        std::cout << "[tray] 行列绑定料盘系：+X前=row1  -X后=row6  -Y右=col1  +Y左=col6\n";
        std::cout << std::fixed << std::setprecision(4)
                  << "[tray] 左右分列按 ArUco 列号，不用机器人 Y=0"
                  << "；盘心 y=" << (std::isfinite(origin_y) ? origin_y : 0.0)
                  << (std::isfinite(origin_y) ? "（基座）" : "（未知）")
                  << (cols_flipped ? "；列号已对调(盘约转180°)\n" : "\n");
        if (!ranked.empty())
        {
            const int r1 = tray_to_from_robot.count(1) ? tray_to_from_robot[1] : 0;
            if (r1 == static_cast<int>(ranked.size()) && ranked.size() >= 2)
                std::cout << "[tray] 料盘相对机器人约转了 180°：ArUco row1 现在离机器人最近，"
                             "抓取姿态改按基座X最远/最近匹配\n";
            else if (r1 == 1)
                std::cout << "[tray] 料盘朝向正常：ArUco row1 离机器人最远\n";
            for (const auto &p : ranked)
            {
                const int fr = tray_to_from_robot[p.second];
                const char *tag = (fr == 1) ? "最远" :
                    (fr == static_cast<int>(ranked.size()) ? "最近" : "中间");
                std::cout << std::fixed << std::setprecision(4)
                          << "  ArUco row" << p.second << " 基座x=" << p.first
                          << " → " << tag << " from_robot=" << fr << "\n";
            }
        }

        auto from_robot_row = [&](int tray_row) -> int {
            const auto it = tray_to_from_robot.find(tray_row);
            return it != tray_to_from_robot.end() ? it->second : tray_row;
        };

        std::vector<Cand> right_zone;
        std::vector<Cand> left_zone;
        for (const TrayHoleResult &h : tray.holes)
        {
            if (h.class_id != want_class)
                continue;
            const double x = std::isfinite(h.x_level) ? h.x_level : h.x;
            const double y = std::isfinite(h.y_level) ? h.y_level : h.y;
            if (!h.in_robot || !std::isfinite(x) || !std::isfinite(y))
                continue;
            if (x < valid.x_min || x > valid.x_max)
            {
                std::cout << "  hole " << h.id << " 过滤(工作包络x) x=" << x << " y=" << y << "\n";
                continue;
            }
            const int col = tray_assign_col_from_aruco(h.col, cols_flipped);
            if (col < 1)
            {
                std::cout << "  hole " << h.id << " 过滤(无 ArUco 列) col=" << h.col << "\n";
                continue;
            }
            
            const bool is_right_col = col <= right_max_col;
            if (is_right_col
                    ? (y < valid.right_y_min || y > valid.right_y_max)
                    : (y < valid.left_y_min || y > valid.left_y_max))
            {
                std::cout << "  hole " << h.id << " 过滤(工作包络y) x=" << x << " y=" << y
                          << (is_right_col ? " 右\n" : " 左\n");
                continue;
            }
            const double z = std::isfinite(h.top_z_level)
                                 ? h.top_z_level
                                 : (std::isfinite(h.top_z) ? h.top_z : expected_object_top_z());
            Cand c;
            c.x = x;
            c.y = y;
            c.z = z;
            c.id = h.id;
            c.col = col;
            c.row = from_robot_row(h.row);
            c.row_aruco = h.row;
            c.col_aruco = h.col;
            c.conf = h.conf;
            if (col <= right_max_col)
                right_zone.push_back(c);
            else
                left_zone.push_back(c);
            std::cout << std::fixed << std::setprecision(4)
                      << "  hole " << h.id << " ArUco row=" << h.row << " col=" << h.col
                      << " from_robot=" << c.row << " 分配col=" << col
                      << (col <= right_max_col ? " 右" : " 左")
                      << " xy=(" << x << "," << y << ") top_z=" << z
                      << " conf=" << std::setprecision(3) << h.conf << "\n";
        }

        auto zone_has_near = [](const std::vector<Cand> &zone) {
            for (const Cand &c : zone)
            {
                if (!tray_row_is_far(c.row))
                    return true;
            }
            return false;
        };
        const bool skip_r = skip_right || right_arm_motors_locked();
        const bool skip_l = skip_left;
        if (skip_r && skip_l)
        {
            std::cout << "[tray_xy] 左右手都已持件，不再分配\n";
            return false;
        }
        if (skip_r)
            std::cout << "[tray_xy] 右手已持件或锁定，本轮只给左手补抓\n";
        if (skip_l)
            std::cout << "[tray_xy] 左手已持件，本轮只给右手补抓\n";

        auto zone_has_far = [](const std::vector<Cand> &zone) {
            for (const Cand &c : zone)
            {
                if (tray_row_is_far(c.row))
                    return true;
            }
            return false;
        };
        const bool near_present =
            (!skip_l && zone_has_near(left_zone)) || (!skip_r && zone_has_near(right_zone));
        const bool far_present =
            (!skip_l && zone_has_far(left_zone)) || (!skip_r && zone_has_far(right_zone));
        // 料盘一抓取：近区再有件则清记忆；近空且远还有件则记住，后续轮次直接远排姿态。
        if (tray_hole_task() == TrayHoleTask::GraspParts &&
            g_move_cfg.grasp_zone.far_row_max > 0)
        {
            if (near_present)
                tray1_far_band_clear();
            else if (far_present)
                tray1_far_band_set(true);
        }
        const bool prefer_near = near_present;
        if (g_move_cfg.grasp_zone.far_row_max > 0)
        {
            if (prefer_near)
                std::cout << "[tray_xy] 3+3 近区优先：先抓离机器人近的三排"
                          << "（from_robot " << (g_move_cfg.grasp_zone.far_row_max + 1)
                          << "-6），远三排暂缓\n";
            else
                std::cout << "[tray_xy] 3+3 近区已空，开始离机器人远的三排 from_robot 1-"
                          << g_move_cfg.grasp_zone.far_row_max
                          << (tray1_far_band_active() ? "（远三排记忆已开）\n" : "\n");
        }

        auto in_prefer_band = [&](const Cand &c) {
            return !(prefer_near && tray_row_is_far(c.row));
        };
        auto leftmost_of = [](const std::vector<const Cand *> &zs) -> const Cand * {
            const Cand *best = nullptr;
            for (const Cand *c : zs)
            {
                if (c == nullptr)
                    continue;
                if (best == nullptr || c->col > best->col ||
                    (c->col == best->col && c->y > best->y))
                    best = c;
            }
            return best;
        };

        // 左右锁在同一行：先清空离机器人最近的一行。满排配对 (3,6)→(2,5)→(1,4)。
        // 一侧本行没料就等，禁止跳去更远行把配对手抽走。
        int shared_row = 0;
        if (g_assign_fixed_from_robot_row >= 1)
        {
            const int want = g_assign_fixed_from_robot_row;
            bool any = false;
            for (const Cand &c : right_zone)
            {
                if (!skip_r && c.row == want)
                    any = true;
            }
            for (const Cand &c : left_zone)
            {
                if (!skip_l && c.row == want)
                    any = true;
            }
            shared_row = any ? want : 0;
            std::cout << "[tray_xy] 指定 from_robot=" << want
                      << (want == 6 ? "（离机器人最近）" : "")
                      << (want == 5 ? "（离机器人第二近）" : "")
                      << (any ? "\n" : "，这一排没有孔\n");
        }
        else
        {
            for (const Cand &c : right_zone)
            {
                if (!skip_r && in_prefer_band(c))
                    shared_row = std::max(shared_row, c.row);
            }
            for (const Cand &c : left_zone)
            {
                if (!skip_l && in_prefer_band(c))
                    shared_row = std::max(shared_row, c.row);
            }
        }

        std::vector<const Cand *> rights;
        std::vector<const Cand *> lefts;
        if (shared_row >= 1)
        {
            for (const Cand &c : right_zone)
            {
                if (!skip_r && in_prefer_band(c) && c.row == shared_row)
                    rights.push_back(&c);
            }
            for (const Cand &c : left_zone)
            {
                if (!skip_l && in_prefer_band(c) && c.row == shared_row)
                    lefts.push_back(&c);
            }
        }

        const Cand *pr = nullptr;
        const Cand *pl = nullptr;
        const int min_delta = g_move_cfg.grasp_zone.min_simultaneous_col_delta;
        if (skip_r)
        {
            pl = leftmost_of(lefts);
        }
        else if (skip_l)
        {
            pr = leftmost_of(rights);
        }
        else if (lefts.empty())
        {
            pr = leftmost_of(rights);
        }
        else if (rights.empty())
        {
            pl = leftmost_of(lefts);
        }
        else
        {
            for (const Cand *l : lefts)
            {
                for (const Cand *r : rights)
                {
                    if ((l->col - r->col) < min_delta)
                        continue;
                    const bool better =
                        pl == nullptr || l->col > pl->col ||
                        (l->col == pl->col && r->col > pr->col);
                    if (better)
                    {
                        pl = l;
                        pr = r;
                    }
                }
            }
            if (pl == nullptr)
            {
                pl = leftmost_of(lefts);
                std::cout << "[tray_xy] from_robot=" << shared_row
                          << " 本行列差不足同时，同行从左往右先抓左\n";
            }
        }
        if (shared_row >= 1)
        {
            std::cout << "[tray_xy] 选孔：同行 from_robot=" << shared_row
                      << " 从左往右；满排同时对 (3,6)/(2,5)/(1,4)"
                      << " 本行右" << rights.size() << "孔 左" << lefts.size() << "孔\n";
            if (pr != nullptr && pl == nullptr && !skip_l)
                std::cout << "[tray_xy] 本行仅右侧有料，左手等本行清空，不抢下一行\n";
            if (pl != nullptr && pr == nullptr && !skip_r && !rights.empty())
                std::cout << "[tray_xy] 本行先抓左，右手本轮停\n";
        }
        if (pr != nullptr)
        {
            st.right_hand_pos[0] = pr->x;
            st.right_hand_pos[1] = pr->y;
            st.right_hand_pos[2] = pr->z;
            st.row_r = pr->row;
            st.row_aruco_r = pr->row_aruco;
            st.col_r = pr->col;
            st.col_aruco_r = pr->col_aruco;
            st.zones.r_from_side = true;
            std::cout << std::fixed << std::setprecision(4)
                      << "[tray_xy] 右手 hole " << pr->id
                      << " ArUco row=" << pr->row_aruco << " col=" << pr->col_aruco
                      << " from_robot=" << pr->row << " 分配col=" << pr->col
                      << " xy=(" << pr->x << "," << pr->y << ")\n";
        }
        if (pl != nullptr)
        {
            st.left_hand_pos[0] = pl->x;
            st.left_hand_pos[1] = pl->y;
            st.left_hand_pos[2] = pl->z;
            st.row_l = pl->row;
            st.row_aruco_l = pl->row_aruco;
            st.col_l = pl->col;
            st.col_aruco_l = pl->col_aruco;
            st.zones.l_from_side = true;
            std::cout << std::fixed << std::setprecision(4)
                      << "[tray_xy] 左手 hole " << pl->id
                      << " ArUco row=" << pl->row_aruco << " col=" << pl->col_aruco
                      << " from_robot=" << pl->row << " 分配col=" << pl->col
                      << " xy=(" << pl->x << "," << pl->y << ")\n";
        }
        st.xy_from_tray = (pr != nullptr) || (pl != nullptr);
        return st.xy_from_tray;
    }

void fill_head_assign_from_frame(
        RealSenseMultiCam &cameras,
        SegPoseBridge &bridge,
        const std::array<double, 16> &cam2robot,
        SegEngineId engine_id,
        HeadAssignState &st,
        bool skip_right,
        bool skip_left)
    {
        st = {};
        if (app_stop_requested() || hardware_abort_requested())
            return;

        std::array<double, 16> T = cam2robot;
        std::string ext_err;
        if (!load_head_cam2robot(T, ext_err))
        {
            std::cerr << "[head_ext] 现算失败: " << ext_err << "，用传入外参\n";
            T = cam2robot;
        }
        (void)T;

        const bool tray_ready = bridge.tray_engine_ready();
        PerceiveTrayOptions opts;
        opts.run_yolo = true;
        opts.tray2_markers = tray2_place_active();
        opts.show_tray_visualization = kDebugVisualize;
        opts.show_yolo_visualization = !tray_ready && kDebugVisualize;
        opts.engine_id = engine_id;
        const HeadTrayPerception perc = perceive_tray_head(cameras, bridge, opts);
        if (perc.frames.empty())
        {
            std::cerr << "[head] 取帧失败: 多帧融合没有有效帧\n";
            return;
        }
        if (opts.run_yolo && !perc.yolo.ok)
            std::cerr << "[head] YOLO 失败: " << perc.yolo.message << std::endl;

        const TrayDetectResult &tray = perc.tray;
        if (!tray_ready)
            std::cerr << "[tray_xy] 料盘引擎未就绪，不回退 YOLO，本轮不抓\n";
        else if (!tray.ok)
            std::cerr << "[tray_xy] 料盘解算失败: " << tray.message
                      << "，不回退 YOLO，本轮不抓\n";

        if (!assign_from_tray_holes(tray, st, skip_right, skip_left))
        {
            if (tray.ok)
            {
                std::cout << "[tray_xy] 料盘已解算，"
                          << (right_arm_motors_locked() ? "左三列" : "本侧列")
                          << (g_tray_hole_task == TrayHoleTask::PlaceEmpty
                                  ? "没有空孔，不放\n"
                                  : "没有毛坯，不抓\n");
            }
            st.xy_from_tray = false;
        }
        if (tray.ok)
        {
            const TrayZoneCount zone = count_zone_from_tray(tray);
            st.zone_count_ok = zone.ok;
            st.zone_right = zone.right;
            st.zone_left = zone.left;
        }
        fill_goal_last_from_head_assign(
            st.right_hand_pos, st.left_hand_pos, st.row_r, st.row_l,
            st.goal_last_r, st.goal_last_l);
        refresh_head_move_flags(st);
    }

HeadAssignState detect_and_assign_head(
        RealSenseMultiCam &cameras,
        SegPoseBridge &bridge,
        const std::array<double, 16> &cam2robot,
        bool retry_if_incomplete,
        SegEngineId engine_id,
        bool skip_right,
        bool skip_left)
    {
        t170::skills::detail::report(t170::skills::Stage::Observe,!skip_right,!skip_left);
        HeadAssignState st;
        if (kDebugVisualize)
            pose_vis_begin_phase(PoseVisLayout::Single, PoseVisPanel::Head, {PoseVisPanel::Head});
        fill_head_assign_from_frame(
            cameras, bridge, cam2robot, engine_id, st, skip_right, skip_left);

        const bool need_r = !skip_right && !right_arm_motors_locked();
        const bool need_l = !skip_left;
        const bool miss_r = need_r && !st.have_r;
        const bool miss_l = need_l && !st.have_l;
        const bool any_assigned = (need_r && st.have_r) || (need_l && st.have_l);
        const bool zone_has_target =
            (miss_r && st.zone_right > 0) || (miss_l && st.zone_left > 0);
        const bool retry = retry_if_incomplete && (miss_r || miss_l) &&
                           (!st.zone_count_ok || (!any_assigned && zone_has_target));
        if (retry_if_incomplete && (miss_r || miss_l) && !retry)
        {
            cout << "[head] 未分配手(右=" << (miss_r ? "无" : "有/不需")
                 << " 左=" << (miss_l ? "无" : "有/不需") << ")"
                 << (any_assigned ? "，另一只手已有目标" : "")
                 << "，料盘已解算 本侧剩余 右=" << st.zone_right << " 左=" << st.zone_left
                 << "，不重拍\n";
        }
        if (retry)
        {
            cout << "[head] 未分配到可动目标(右="
                 << (st.have_r ? "有" : "无") << " 左=" << (st.have_l ? "有" : "无")
                 << (skip_right ? " 右手已持件" : "")
                 << (skip_left ? " 左手已持件" : "")
                 << (right_arm_motors_locked() ? " 右臂锁定" : "")
                 << (st.zone_count_ok ? "" : " 料盘未解算")
                 << ")，头相机重试一次\n";
            if (kDebugVisualize)
                pose_vis_clear_panels({PoseVisPanel::Head});
            fill_head_assign_from_frame(
                cameras, bridge, cam2robot, engine_id, st, skip_right, skip_left);
        }
        return st;
    }


}
