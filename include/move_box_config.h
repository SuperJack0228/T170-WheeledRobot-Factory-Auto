#pragma once

// Compatibility facade: configuration types/loading now live in common;
// target-selection policy remains here until the skill migration.
#include "t170/common/config.h"

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
/** 料盘2放置统一使用第五行（ready2）的 RPY，所有行/ready 共用，不改抓取姿态。 */
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
