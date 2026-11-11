// Legacy target-selection and row/offset policy.
// Parameter definitions and YAML loading moved to src/common/config.cpp.
#include "move_box_config.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

namespace
{
bool g_have_live_tray_origin_y = false;
double g_live_tray_origin_y = 0.0;
} // namespace

double column_split_y()
{
    if (g_have_live_tray_origin_y && std::isfinite(g_live_tray_origin_y))
        return g_live_tray_origin_y;
    return g_move_cfg.grasp_zone.column_split_y;
}

void set_live_tray_origin_y(double origin_y)
{
    if (!std::isfinite(origin_y))
    {
        g_have_live_tray_origin_y = false;
        return;
    }
    g_have_live_tray_origin_y = true;
    g_live_tray_origin_y = origin_y;
}

void clear_live_tray_origin_y()
{
    g_have_live_tray_origin_y = false;
}

bool have_live_tray_origin_y()
{
    return g_have_live_tray_origin_y && std::isfinite(g_live_tray_origin_y);
}

bool tray_row_is_far(int row)
{
    const int n = g_move_cfg.grasp_zone.far_row_max;
    return n > 0 && row >= 1 && row <= n;
}

int tray_row_pose_group(int row)
{
    if (row < 1)
        return 0;
    return 2 - ((row - 1) % 3);
}

const GraspRpyDeg &grasp_rpy_deg_for_row(int row, bool is_right)
{
    if (row == 1)
        return is_right ? g_move_cfg.head_grasp.right_row6_rpy_deg
                        : g_move_cfg.head_grasp.left_row6_rpy_deg;
    const int g = tray_row_pose_group(row);
    return is_right ? g_move_cfg.head_grasp.right_row_rpy_deg[g]
                    : g_move_cfg.head_grasp.left_row_rpy_deg[g];
}

const GraspRpyDeg &grasp_rpy_deg_for_ready(int ready_id, bool is_right)
{
    if (ready_id == 6)
        return is_right ? g_move_cfg.head_grasp.right_row6_rpy_deg
                        : g_move_cfg.head_grasp.left_row6_rpy_deg;
    const int g = std::clamp(ready_id, 1, 3) - 1;
    return is_right ? g_move_cfg.head_grasp.right_row_rpy_deg[g]
                    : g_move_cfg.head_grasp.left_row_rpy_deg[g];
}

const GraspRpyDeg &tray2_place_rpy_deg_for_row(int row, bool is_right)
{
    (void)row;
    return is_right ? g_move_cfg.tray2_place.right_row_rpy_deg[1]
                    : g_move_cfg.tray2_place.left_row_rpy_deg[1];
}

const GraspRpyDeg &tray2_place_rpy_deg_for_ready(int ready_id, bool is_right)
{
    (void)ready_id;
    return tray2_place_rpy_deg_for_row(5,is_right);
}

const char *ready_rows_label(int ready_id)
{
    switch (ready_id)
    {
    case 1:
        return "row3/6（近）";
    case 2:
        return "row2/5";
    case 3:
        return "row4（row1最远另有专用）";
    case 6:
        return "仅最远row1";
    default:
        return "?";
    }
}

int tray_assign_col_from_aruco(int aruco_col, bool cols_flipped)
{
    const int n = g_move_cfg.grasp_zone.column_count;
    if (aruco_col < 1 || n < 1 || aruco_col > n)
        return 0;
    return cols_flipped ? (n + 1 - aruco_col) : aruco_col;
}

bool tray_assign_col_is_right(int assign_col)
{
    const int n = g_move_cfg.grasp_zone.column_count;
    const int right_max = n / 2;
    return assign_col >= 1 && right_max >= 1 && assign_col <= right_max;
}

int tray_column_index_from_y(double y)
{
    const auto &z = g_move_cfg.grasp_zone;
    if (!std::isfinite(y) || z.hole_pitch_m <= 1e-9 || z.column_count < 1)
        return 0;
    // 列 1..N，+Y 为左。列中心 = 盘心/中缝 + (col - (N+1)/2) * pitch
    const double split = column_split_y();
    const double col = (y - split) / z.hole_pitch_m + 0.5 * (z.column_count + 1);
    int i = static_cast<int>(std::lround(col));
    if (i < 1)
        i = 1;
    if (i > z.column_count)
        i = z.column_count;
    // lround(3.5)=4，但 y<=盘心 归属右手，必须落在 1..N/2，否则中缝会和列号打架。
    const int right_max = z.column_count / 2;
    if (right_max >= 1)
    {
        if (y <= split + 1e-9)
        {
            if (i > right_max)
                i = right_max;
        }
        else if (i <= right_max)
            i = right_max + 1;
    }
    return i;
}

double grasp_goal_x_offset(bool is_right)
{
    return is_right ? g_move_cfg.head_grasp.right_goal_x_offset
                    : g_move_cfg.head_grasp.goal_x_offset;
}

double grasp_goal_y_offset(bool is_right)
{
    return is_right ? g_move_cfg.head_grasp.right_goal_y_offset
                    : g_move_cfg.head_grasp.goal_y_offset;
}

double grasp_goal_z_offset(bool is_right, int from_robot_row)
{
    const double z = is_right ? g_move_cfg.head_grasp.right_grasp_z_offset_m
                              : g_move_cfg.head_grasp.grasp_z_offset_m;
    if (from_robot_row == 6)
        return z + g_move_cfg.head_grasp.nearest_row_grasp_z_offset_m;
    return z;
}

double waist_ready_x()
{
    return g_move_cfg.waist.layer3_home(0) + g_move_cfg.waist.ready_forward_m;
}

double waist_ready_z()
{
    return g_move_cfg.waist.ready_z_m;
}

double waist_far_row_x()
{
    return g_move_cfg.waist.layer3_home(0) + g_move_cfg.waist.far_row_forward_m;
}

void conveyor_chassis_xy_to_arm_base(double yaw_rad, double &x, double &y)
{
    if (!std::isfinite(yaw_rad) || std::abs(yaw_rad) < 1e-12)
        return;
    const double c = std::cos(-yaw_rad);
    const double s = std::sin(-yaw_rad);
    const double xn = c * x - s * y;
    const double yn = s * x + c * y;
    x = xn;
    y = yn;
}

bool arm_y_allowed_right(double y)
{
    return std::isfinite(y) && y <= column_split_y() + 1e-9;
}

bool arm_y_allowed_left(double y)
{
    return std::isfinite(y) && y > column_split_y() + 1e-9;
}

bool simultaneous_assign_columns_ok(int col_right, int col_left)
{
    if (col_right < 1 || col_left < 1)
        return false;
    if (!tray_assign_col_is_right(col_right) || tray_assign_col_is_right(col_left))
        return false;
    return (col_left - col_right) >= g_move_cfg.grasp_zone.min_simultaneous_col_delta;
}

bool simultaneous_columns_ok(double y_right, double y_left)
{
    if (!arm_y_allowed_right(y_right) || !arm_y_allowed_left(y_left))
        return false;
    return simultaneous_assign_columns_ok(
        tray_column_index_from_y(y_right), tray_column_index_from_y(y_left));
}

void log_arm_wall_reject(const char *stage, bool is_right, double y)
{
    const double split = column_split_y();
    const int col = tray_column_index_from_y(y);
    std::cout << std::fixed << std::setprecision(4)
              << "[col] " << (is_right ? "右" : "左") << " " << stage
              << " y=" << y << " col=" << col
              << (is_right ? " 无料盘且越过盘心 y>" : " 无料盘且越过盘心 y<=") << split
              << "，拒绝本侧，不 clamp\n";
}
