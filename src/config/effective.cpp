// Explicit runtime member snapshot. Includes defaults, derived fields and loaded calibration.
#include "t170/config/effective.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <type_traits>
namespace t170::config {
namespace {
template<class T> requires (std::is_arithmetic_v<T> || std::is_same_v<T,std::string>)
YAML::Node encode(const T &v) {return YAML::Node(v);}
template<class D> YAML::Node encode(const Eigen::MatrixBase<D> &v) {
 YAML::Node n(YAML::NodeType::Sequence);
 for(int i=0;i<v.rows();++i) {YAML::Node row(YAML::NodeType::Sequence);for(int j=0;j<v.cols();++j)row.push_back(v(i,j));n.push_back(row);}return n;
}
YAML::Node encode(const ArmPosePairConfig &);
YAML::Node encode(const HandXyOffsetConfig &);
YAML::Node encode(const GraspRpyDeg &);
YAML::Node encode(const XyzOffsetM &);
YAML::Node encode(const TrayHeightConfig &);
YAML::Node encode(const HeadGraspConfig &);
YAML::Node encode(const GraspValidConfig &);
YAML::Node encode(const GraspZoneConfig &);
YAML::Node encode(const WaistConfig &);
YAML::Node encode(const ConveyorXyzOffset &);
YAML::Node encode(const ChassisConfig &);
YAML::Node encode(const Tray2PlaceConfig &);
YAML::Node encode(const ConveyorStationConfig &);
YAML::Node encode(const StaggerConfig &);
YAML::Node encode(const GripperCheckConfig &);
YAML::Node encode(const VisionDetectConfig &);
YAML::Node encode(const CameraStreamConfig &);
YAML::Node encode(const HeadMotorConfig &);
YAML::Node encode(const RetractMotionConfig &);
YAML::Node encode(const MoveBoxConfig &);
YAML::Node encode(const wrist_servo::MapKnot &);
YAML::Node encode(const wrist_servo::ImageProfile &);
YAML::Node encode(const wrist_servo::PlacementCalibrationProfile &);
YAML::Node encode(const wrist_servo::Config &);
YAML::Node encode(const wrist_servo::TrayPlaceVisionConfig &);
template<class T> YAML::Node encode(const std::vector<T> &v) {YAML::Node n(YAML::NodeType::Sequence);for(const auto &x:v)n.push_back(encode(x));return n;}
template<class T,std::size_t N> YAML::Node encode(const std::array<T,N> &v) {YAML::Node n(YAML::NodeType::Sequence);for(const auto &x:v)n.push_back(encode(x));return n;}
template<class T,std::size_t N> YAML::Node encode(const T (&v)[N]) {YAML::Node n(YAML::NodeType::Sequence);for(const auto &x:v)n.push_back(encode(x));return n;}
YAML::Node encode(const ArmPosePairConfig &v) {
 YAML::Node n;
 n["right"] = encode(v.right);
 n["left"] = encode(v.left);
 return n;
}
YAML::Node encode(const HandXyOffsetConfig &v) {
 YAML::Node n;
 n["offset_x"] = encode(v.offset_x);
 n["offset_y"] = encode(v.offset_y);
 return n;
}
YAML::Node encode(const GraspRpyDeg &v) {
 YAML::Node n;
 n["rx"] = encode(v.rx);
 n["ry"] = encode(v.ry);
 n["rz"] = encode(v.rz);
 return n;
}
YAML::Node encode(const XyzOffsetM &v) {
 YAML::Node n;
 n["x"] = encode(v.x);
 n["y"] = encode(v.y);
 n["z"] = encode(v.z);
 return n;
}
YAML::Node encode(const TrayHeightConfig &v) {
 YAML::Node n;
 n["z_ref_m"] = encode(v.z_ref_m);
 n["z_ref_waist_z_m"] = encode(v.z_ref_waist_z_m);
 n["part_above_tray_m"] = encode(v.part_above_tray_m);
 n["z_refine_max_m"] = encode(v.z_refine_max_m);
 n["fuse_frames"] = encode(v.fuse_frames);
 return n;
}
YAML::Node encode(const HeadGraspConfig &v) {
 YAML::Node n;
 n["hover_above_m"] = encode(v.hover_above_m);
 n["goal_z_base"] = encode(v.goal_z_base);
 n["goal_x_offset"] = encode(v.goal_x_offset);
 n["goal_y_offset"] = encode(v.goal_y_offset);
 n["goal_z_extra"] = encode(v.goal_z_extra);
 n["right_goal_x_offset"] = encode(v.right_goal_x_offset);
 n["right_goal_y_offset"] = encode(v.right_goal_y_offset);
 n["right_grasp_z_offset_m"] = encode(v.right_grasp_z_offset_m);
 n["right_rx_deg"] = encode(v.right_rx_deg);
 n["right_ry_deg"] = encode(v.right_ry_deg);
 n["right_rz_deg"] = encode(v.right_rz_deg);
 n["left_rx_deg"] = encode(v.left_rx_deg);
 n["left_ry_deg"] = encode(v.left_ry_deg);
 n["left_rz_deg"] = encode(v.left_rz_deg);
 n["left_row_rpy_deg"] = encode(v.left_row_rpy_deg);
 n["right_row_rpy_deg"] = encode(v.right_row_rpy_deg);
 n["left_row6_rpy_deg"] = encode(v.left_row6_rpy_deg);
 n["right_row6_rpy_deg"] = encode(v.right_row6_rpy_deg);
 n["approach_nostop"] = encode(v.approach_nostop);
 n["adaptive_pitch_down_deg"] = encode(v.adaptive_pitch_down_deg);
 n["adaptive_yaw_max_deg"] = encode(v.adaptive_yaw_max_deg);
 n["adaptive_min_horiz_m"] = encode(v.adaptive_min_horiz_m);
 n["adaptive_approach_xyz_tol_m"] = encode(v.adaptive_approach_xyz_tol_m);
 n["adaptive_approach_rpy_tol_deg"] = encode(v.adaptive_approach_rpy_tol_deg);
 n["head_approach_z_descend"] = encode(v.head_approach_z_descend);
 n["grasp_z_offset_m"] = encode(v.grasp_z_offset_m);
 n["use_hand_camera"] = encode(v.use_hand_camera);
 n["hand_match_xy_max_m"] = encode(v.hand_match_xy_max_m);
 n["hand_xy_refine_max_m"] = encode(v.hand_xy_refine_max_m);
 n["hand_right"] = encode(v.hand_right);
 n["hand_left"] = encode(v.hand_left);
 n["lift_after_grasp_z"] = encode(v.lift_after_grasp_z);
 n["bezier_endpoint_xyz_tol_m"] = encode(v.bezier_endpoint_xyz_tol_m);
 n["bezier_endpoint_rpy_tol_deg"] = encode(v.bezier_endpoint_rpy_tol_deg);
 n["approach_vel_m_s"] = encode(v.approach_vel_m_s);
 n["descend_vel_m_s"] = encode(v.descend_vel_m_s);
 n["lift_vel_m_s"] = encode(v.lift_vel_m_s);
 n["return_vel_m_s"] = encode(v.return_vel_m_s);
 n["retract_lift_vel_m_s"] = encode(v.retract_lift_vel_m_s);
 n["rise_waist_with_lift"] = encode(v.rise_waist_with_lift);
 n["waist_lift_start_clearance_m"] = encode(v.waist_lift_start_clearance_m);
 n["retract_arrival_tolerance_m"] = encode(v.retract_arrival_tolerance_m);
 n["retract_arrival_timeout_s"] = encode(v.retract_arrival_timeout_s);
 n["retract_connector_speed_rad_s"] = encode(v.retract_connector_speed_rad_s);
 n["retract_connector_accel_rad_s2"] = encode(v.retract_connector_accel_rad_s2);
 n["retract_connector_jerk_rad_s3"] = encode(v.retract_connector_jerk_rad_s3);
 n["retract_connector_max_s"] = encode(v.retract_connector_max_s);
 n["pre_grasp_settle_sec"] = encode(v.pre_grasp_settle_sec);
 n["hand_detect_invalid_redo_max"] = encode(v.hand_detect_invalid_redo_max);
 return n;
}
YAML::Node encode(const GraspValidConfig &v) {
 YAML::Node n;
 n["z_min"] = encode(v.z_min);
 n["z_max"] = encode(v.z_max);
 n["x_min"] = encode(v.x_min);
 n["x_max"] = encode(v.x_max);
 n["right_y_min"] = encode(v.right_y_min);
 n["right_y_max"] = encode(v.right_y_max);
 n["left_y_min"] = encode(v.left_y_min);
 n["left_y_max"] = encode(v.left_y_max);
 return n;
}
YAML::Node encode(const GraspZoneConfig &v) {
 YAML::Node n;
 n["y_max_abs"] = encode(v.y_max_abs);
 n["x_min"] = encode(v.x_min);
 n["x_max"] = encode(v.x_max);
 n["edge_margin_frac"] = encode(v.edge_margin_frac);
 n["cam_xy_over_z_max"] = encode(v.cam_xy_over_z_max);
 n["front_row_tolerance_m"] = encode(v.front_row_tolerance_m);
 n["column_split_y"] = encode(v.column_split_y);
 n["hole_pitch_m"] = encode(v.hole_pitch_m);
 n["column_count"] = encode(v.column_count);
 n["min_simultaneous_col_delta"] = encode(v.min_simultaneous_col_delta);
 n["far_row_max"] = encode(v.far_row_max);
 return n;
}
YAML::Node encode(const WaistConfig &v) {
 YAML::Node n;
 n["layer3_home"] = encode(v.layer3_home);
 n["stagger_step_x"] = encode(v.stagger_step_x);
 n["stagger_max_steps"] = encode(v.stagger_max_steps);
 n["move_settle_sec"] = encode(v.move_settle_sec);
 n["x_min"] = encode(v.x_min);
 n["x_max"] = encode(v.x_max);
 n["z_min"] = encode(v.z_min);
 n["z_max"] = encode(v.z_max);
 n["grasp_object_z_min"] = encode(v.grasp_object_z_min);
 n["grasp_lower_z"] = encode(v.grasp_lower_z);
 n["ready_z_m"] = encode(v.ready_z_m);
 n["ready_forward_m"] = encode(v.ready_forward_m);
 n["far_row_forward_m"] = encode(v.far_row_forward_m);
 return n;
}
YAML::Node encode(const ConveyorXyzOffset &v) {
 YAML::Node n;
 n["x"] = encode(v.x);
 n["y"] = encode(v.y);
 n["z"] = encode(v.z);
 return n;
}
YAML::Node encode(const ChassisConfig &v) {
 YAML::Node n;
 n["host"] = encode(v.host);
 n["robot_type"] = encode(v.robot_type);
 n["sequence_number"] = encode(v.sequence_number);
 n["tray_station"] = encode(v.tray_station);
 n["dispatch_wait_station"] = encode(v.dispatch_wait_station);
 n["charge_station"] = encode(v.charge_station);
 n["charge_wait_station"] = encode(v.charge_wait_station);
 n["charge_done_percent"] = encode(v.charge_done_percent);
 n["tray2_station"] = encode(v.tray2_station);
 n["belt_station"] = encode(v.belt_station);
 n["out_station"] = encode(v.out_station);
 n["nav_timeout_ms"] = encode(v.nav_timeout_ms);
 return n;
}
YAML::Node encode(const Tray2PlaceConfig &v) {
 YAML::Node n;
 n["retract_lift_m"] = encode(v.retract_lift_m);
 n["retract_lift_vel_m_s"] = encode(v.retract_lift_vel_m_s);
 n["retract_return_vel_m_s"] = encode(v.retract_return_vel_m_s);
 n["pre_open_settle_sec"] = encode(v.pre_open_settle_sec);
 n["open_tolerance_rad"] = encode(v.open_tolerance_rad);
 n["open_timeout_ms"] = encode(v.open_timeout_ms);
 n["release_q_rad"] = encode(v.release_q_rad);
 n["left_row_rpy_deg"] = encode(v.left_row_rpy_deg);
 n["right_row_rpy_deg"] = encode(v.right_row_rpy_deg);
 n["left_row6_rpy_deg"] = encode(v.left_row6_rpy_deg);
 n["right_row6_rpy_deg"] = encode(v.right_row6_rpy_deg);
 n["goal_x_offset"] = encode(v.goal_x_offset);
 n["goal_y_offset"] = encode(v.goal_y_offset);
 n["grasp_z_offset_m"] = encode(v.grasp_z_offset_m);
 n["right_goal_x_offset"] = encode(v.right_goal_x_offset);
 n["right_goal_y_offset"] = encode(v.right_goal_y_offset);
 n["right_grasp_z_offset_m"] = encode(v.right_grasp_z_offset_m);
 n["far_row_forward_m"] = encode(v.far_row_forward_m);
 return n;
}
YAML::Node encode(const ConveyorStationConfig &v) {
 YAML::Node n;
 n["waist_x"] = encode(v.waist_x);
 n["waist_z"] = encode(v.waist_z);
 n["head_yaw_deg"] = encode(v.head_yaw_deg);
 n["head_pitch_deg"] = encode(v.head_pitch_deg);
 n["head_roll_deg"] = encode(v.head_roll_deg);
 n["tcp"] = encode(v.tcp);
 n["place_tcp"] = encode(v.place_tcp);
 n["offset_right"] = encode(v.offset_right);
 n["offset_left"] = encode(v.offset_left);
 n["prefer_belt"] = encode(v.prefer_belt);
 n["grasp_rpy_left"] = encode(v.grasp_rpy_left);
 n["grasp_rpy_right"] = encode(v.grasp_rpy_right);
 n["grasp_offset_left"] = encode(v.grasp_offset_left);
 n["grasp_offset_right"] = encode(v.grasp_offset_right);
 n["grasp_tcp"] = encode(v.grasp_tcp);
 n["grasp_hover_above_m"] = encode(v.grasp_hover_above_m);
 n["place_hover_above_m"] = encode(v.place_hover_above_m);
 n["place_approach_vel_m_s"] = encode(v.place_approach_vel_m_s);
 n["place_retract_vel_m_s"] = encode(v.place_retract_vel_m_s);
 n["grasp_approach_vel_m_s"] = encode(v.grasp_approach_vel_m_s);
 n["grasp_retract_vel_m_s"] = encode(v.grasp_retract_vel_m_s);
 n["grasp_descend_vel_m_s"] = encode(v.grasp_descend_vel_m_s);
 n["next_hand_clearance_m"] = encode(v.next_hand_clearance_m);
 n["next_hand_clearance_timeout_s"] = encode(v.next_hand_clearance_timeout_s);
 n["place_base_z_bias_m"] = encode(v.place_base_z_bias_m);
 n["grasp_waist_yaw_deg"] = encode(v.grasp_waist_yaw_deg);
 n["grasp_ready_close_ratio"] = encode(v.grasp_ready_close_ratio);
 n["yolo_roi_enable"] = encode(v.yolo_roi_enable);
 n["yolo_min_conf"] = encode(v.yolo_min_conf);
 n["yolo_part_class_ids"] = encode(v.yolo_part_class_ids);
 n["use_fixed_origin_z"] = encode(v.use_fixed_origin_z);
 n["fixed_origin_z_m"] = encode(v.fixed_origin_z_m);
 n["fixed_origin_ref_waist_z_m"] = encode(v.fixed_origin_ref_waist_z_m);
 n["photo_after_arrival_sec"] = encode(v.photo_after_arrival_sec);
 n["grasp_photo_settle_sec"] = encode(v.grasp_photo_settle_sec);
 return n;
}
YAML::Node encode(const StaggerConfig &v) {
 YAML::Node n;
 n["head_x_threshold"] = encode(v.head_x_threshold);
 return n;
}
YAML::Node encode(const GripperCheckConfig &v) {
 YAML::Node n;
 n["enable"] = encode(v.enable);
 n["empty_q_rad"] = encode(v.empty_q_rad);
 n["hold_q_max_rad"] = encode(v.hold_q_max_rad);
 n["max_empty_retry"] = encode(v.max_empty_retry);
 return n;
}
YAML::Node encode(const VisionDetectConfig &v) {
 YAML::Node n;
 n["head_grasp"] = encode(v.head_grasp);
 n["right_hand_grasp"] = encode(v.right_hand_grasp);
 n["left_hand_grasp"] = encode(v.left_hand_grasp);
 return n;
}
YAML::Node encode(const CameraStreamConfig &v) {
 YAML::Node n;
 n["width"] = encode(v.width);
 n["height"] = encode(v.height);
 n["head_fps"] = encode(v.head_fps);
 n["hand_fps"] = encode(v.hand_fps);
 n["hand_live_preview"] = encode(v.hand_live_preview);
 n["hand_descent_correct"] = encode(v.hand_descent_correct);
 n["left_gripper_u"] = encode(v.left_gripper_u);
 n["left_gripper_v"] = encode(v.left_gripper_v);
 n["right_gripper_u"] = encode(v.right_gripper_u);
 n["right_gripper_v"] = encode(v.right_gripper_v);
 return n;
}
YAML::Node encode(const HeadMotorConfig &v) {
 YAML::Node n;
 n["yaw_deg"] = encode(v.yaw_deg);
 n["pitch_deg"] = encode(v.pitch_deg);
 n["far_pitch_deg"] = encode(v.far_pitch_deg);
 n["roll_deg"] = encode(v.roll_deg);
 n["speed_deg_s"] = encode(v.speed_deg_s);
 return n;
}
YAML::Node encode(const RetractMotionConfig &v) {
 YAML::Node n;
 n["ready_right"] = encode(v.ready_right);
 n["ready_left"] = encode(v.ready_left);
 n["max_joint_velocity_rad_s"] = encode(v.max_joint_velocity_rad_s);
 n["max_joint_acceleration_rad_s2"] = encode(v.max_joint_acceleration_rad_s2);
 n["max_joint_jerk_rad_s3"] = encode(v.max_joint_jerk_rad_s3);
 n["arrival_joint_tolerance_rad"] = encode(v.arrival_joint_tolerance_rad);
 n["settle_timeout_s"] = encode(v.settle_timeout_s);
 n["feedback_timeout_s"] = encode(v.feedback_timeout_s);
 n["stable_samples"] = encode(v.stable_samples);
 return n;
}
YAML::Node encode(const MoveBoxConfig &v) {
 YAML::Node n;
 n["standby"] = encode(v.standby);
 n["home_tcp"] = encode(v.home_tcp);
 n["tray"] = encode(v.tray);
 n["head"] = encode(v.head);
 n["head_grasp"] = encode(v.head_grasp);
 n["retract_motion"] = encode(v.retract_motion);
 n["grasp_valid"] = encode(v.grasp_valid);
 n["grasp_zone"] = encode(v.grasp_zone);
 n["waist"] = encode(v.waist);
 n["conveyor"] = encode(v.conveyor);
 n["conveyor2"] = encode(v.conveyor2);
 n["tray2_place"] = encode(v.tray2_place);
 n["chassis"] = encode(v.chassis);
 n["stagger"] = encode(v.stagger);
 n["vision_detect"] = encode(v.vision_detect);
 n["cameras"] = encode(v.cameras);
 n["visual_servo"] = encode(v.visual_servo);
 n["gripper_check"] = encode(v.gripper_check);
 return n;
}
YAML::Node encode(const wrist_servo::MapKnot &v) {
 YAML::Node n;
 n["height_m"] = encode(v.height_m);
 n["reference_px"] = encode(v.reference_px);
 n["image_jacobian"] = encode(v.image_jacobian);
 return n;
}
YAML::Node encode(const wrist_servo::ImageProfile &v) {
 YAML::Node n;
 n["mode"] = encode(v.mode);
 n["reference_px"] = encode(v.reference_px);
 n["reference_height_m"] = encode(v.reference_height_m);
 n["plane_relative_reference"] = encode(v.plane_relative_reference);
 n["constrain_reference_tilt"] = encode(v.constrain_reference_tilt);
 n["reference_roll_rad"] = encode(v.reference_roll_rad);
 n["reference_pitch_rad"] = encode(v.reference_pitch_rad);
 n["reference_tcp_above_plane_m"] = encode(v.reference_tcp_above_plane_m);
 n["feature_z_offset_m"] = encode(v.feature_z_offset_m);
 n["min_depth_m"] = encode(v.min_depth_m);
 n["max_depth_m"] = encode(v.max_depth_m);
 n["knots"] = encode(v.knots);
 return n;
}
YAML::Node encode(const wrist_servo::PlacementCalibrationProfile &v) {
 YAML::Node n;
 n["image"] = encode(v.image);
 n["head_xy_bias"] = encode(v.head_xy_bias);
 n["universal"] = encode(v.universal);
 return n;
}
YAML::Node encode(const wrist_servo::Config &v) {
 YAML::Node n;
 n["calibrated_placement"] = encode(v.calibrated_placement);
 n["calibrated_tray_grasp"] = encode(v.calibrated_tray_grasp);
 n["belt_place_enabled"] = encode(v.belt_place_enabled);
 n["belt_grasp_enabled"] = encode(v.belt_grasp_enabled);
 n["require_dual"] = encode(v.require_dual);
 n["tray_place"] = encode(v.tray_place);
 n["period_s"] = encode(v.period_s);
 n["kp"] = encode(v.kp);
 n["kd"] = encode(v.kd);
 n["max_xy_speed_m_s"] = encode(v.max_xy_speed_m_s);
 n["max_xy_accel_m_s2"] = encode(v.max_xy_accel_m_s2);
 n["max_z_accel_m_s2"] = encode(v.max_z_accel_m_s2);
 n["brake_delay_s"] = encode(v.brake_delay_s);
 n["model_uncertainty_m"] = encode(v.model_uncertainty_m);
 n["xy_tolerance_m"] = encode(v.xy_tolerance_m);
 n["settle_speed_m_s"] = encode(v.settle_speed_m_s);
 n["z_tolerance_m"] = encode(v.z_tolerance_m);
 n["commit_height_m"] = encode(v.commit_height_m);
 n["freeze_height_m"] = encode(v.freeze_height_m);
 n["commit_speed_m_s"] = encode(v.commit_speed_m_s);
 n["commit_timeout_s"] = encode(v.commit_timeout_s);
 n["grasp_height_margin_m"] = encode(v.grasp_height_margin_m);
 n["grasp_z_accept_above_m"] = encode(v.grasp_z_accept_above_m);
 n["grasp_z_feedback_tolerance_m"] = encode(v.grasp_z_feedback_tolerance_m);
 n["grasp_xy_accept_m"] = encode(v.grasp_xy_accept_m);
 n["grasp_relax_after_s"] = encode(v.grasp_relax_after_s);
 n["finish_speed_m_s"] = encode(v.finish_speed_m_s);
 n["finish_stable_s"] = encode(v.finish_stable_s);
 n["place_xy_accept_m"] = encode(v.place_xy_accept_m);
 n["place_lock_height_m"] = encode(v.place_lock_height_m);
 n["place_visibility_floor_m"] = encode(v.place_visibility_floor_m);
 n["place_goal_stability_m"] = encode(v.place_goal_stability_m);
 n["place_lock_frames"] = encode(v.place_lock_frames);
 n["place_finish_speed_m_s"] = encode(v.place_finish_speed_m_s);
 n["place_height_margin_m"] = encode(v.place_height_margin_m);
 n["place_z_accept_above_m"] = encode(v.place_z_accept_above_m);
 n["alignment_time_margin_s"] = encode(v.alignment_time_margin_s);
 n["acquire_descent_m"] = encode(v.acquire_descent_m);
 n["handoff_distance_m"] = encode(v.handoff_distance_m);
 n["handoff_speed_m_s"] = encode(v.handoff_speed_m_s);
 n["max_correction_m"] = encode(v.max_correction_m);
 n["innovation_gate_m"] = encode(v.innovation_gate_m);
 n["goal_filter_alpha"] = encode(v.goal_filter_alpha);
 n["max_observation_age_s"] = encode(v.max_observation_age_s);
 n["max_clock_uncertainty_s"] = encode(v.max_clock_uncertainty_s);
 n["feedback_age_budget_s"] = encode(v.feedback_age_budget_s);
 n["loss_timeout_s"] = encode(v.loss_timeout_s);
 n["task_timeout_s"] = encode(v.task_timeout_s);
 n["max_tick_gap_s"] = encode(v.max_tick_gap_s);
 n["max_state_gap_s"] = encode(v.max_state_gap_s);
 n["max_reference_lead_m"] = encode(v.max_reference_lead_m);
 n["hard_reference_lead_m"] = encode(v.hard_reference_lead_m);
 n["following_timeout_s"] = encode(v.following_timeout_s);
 n["enforce_sdk_joint_limits"] = encode(v.enforce_sdk_joint_limits);
 n["max_joint_excursion_rad"] = encode(v.max_joint_excursion_rad);
 n["max_joint_lead_rad"] = encode(v.max_joint_lead_rad);
 n["max_joint_speed_rad_s"] = encode(v.max_joint_speed_rad_s);
 n["max_joint_accel_rad_s2"] = encode(v.max_joint_accel_rad_s2);
 n["max_orientation_error_rad"] = encode(v.max_orientation_error_rad);
 n["solver_position_tolerance_m"] = encode(v.solver_position_tolerance_m);
 n["solver_rotation_tolerance_rad"] = encode(v.solver_rotation_tolerance_rad);
 n["max_solver_failures"] = encode(v.max_solver_failures);
 n["stable_frames"] = encode(v.stable_frames);
 n["min_confidence"] = encode(v.min_confidence);
 n["belt_min_confidence"] = encode(v.belt_min_confidence);
 n["acquire_radius_px"] = encode(v.acquire_radius_px);
 n["track_radius_px"] = encode(v.track_radius_px);
 n["pair_clearance_m"] = encode(v.pair_clearance_m);
 n["log_period_s"] = encode(v.log_period_s);
 n["profiles"] = encode(v.profiles);
 return n;
}
YAML::Node encode(const wrist_servo::TrayPlaceVisionConfig &v) {
 YAML::Node n;
 n["enabled"] = encode(v.enabled);
 n["board_config"] = encode(v.board_config);
 n["association_gate_m"] = encode(v.association_gate_m);
 n["reprojection_gate_px"] = encode(v.reprojection_gate_px);
 n["circle_residual_m"] = encode(v.circle_residual_m);
 n["max_plane_update_m"] = encode(v.max_plane_update_m);
 n["min_neighbors"] = encode(v.min_neighbors);
 n["rectified_pixels_per_m"] = encode(v.rectified_pixels_per_m);
 n["xy_only"] = encode(v.xy_only);
 n["affine_enabled"] = encode(v.affine_enabled);
 n["observe_at_B"] = encode(v.observe_at_B);
 n["observe_timeout_s"] = encode(v.observe_timeout_s);
 n["observation_age_s"] = encode(v.observation_age_s);
 n["target_hold_s"] = encode(v.target_hold_s);
 n["alignment_extension_s"] = encode(v.alignment_extension_s);
 n["confirm_stable_s"] = encode(v.confirm_stable_s);
 n["tracking_max_neighbors"] = encode(v.tracking_max_neighbors);
 n["tracking_scan_limit"] = encode(v.tracking_scan_limit);
 n["affine_min_scale"] = encode(v.affine_min_scale);
 n["affine_max_scale"] = encode(v.affine_max_scale);
 n["affine_max_rotation_rad"] = encode(v.affine_max_rotation_rad);
 n["max_extrapolation_pitches"] = encode(v.max_extrapolation_pitches);
 n["orange_rim_enabled"] = encode(v.orange_rim_enabled);
 n["occupied_orange_rims"] = encode(v.occupied_orange_rims);
 n["rim_min_radius_ratio"] = encode(v.rim_min_radius_ratio);
 n["rim_max_radius_ratio"] = encode(v.rim_max_radius_ratio);
 n["min_coverage_bins"] = encode(v.min_coverage_bins);
 n["global_acquisition"] = encode(v.global_acquisition);
 n["boundary_enabled"] = encode(v.boundary_enabled);
 n["global_search_m"] = encode(v.global_search_m);
 n["global_match_gate_m"] = encode(v.global_match_gate_m);
 n["global_ambiguity_margin"] = encode(v.global_ambiguity_margin);
 n["global_max_hypotheses"] = encode(v.global_max_hypotheses);
 n["reacquire_after_frames"] = encode(v.reacquire_after_frames);
 n["partial_rims"] = encode(v.partial_rims);
 n["partial_acquire_min_neighbors"] = encode(v.partial_acquire_min_neighbors);
 n["partial_acquire_confirm_frames"] = encode(v.partial_acquire_confirm_frames);
 n["partial_acquire_stability_m"] = encode(v.partial_acquire_stability_m);
 n["partial_min_bins"] = encode(v.partial_min_bins);
 n["single_neighbor_max_frames"] = encode(v.single_neighbor_max_frames);
 n["single_neighbor_step_m"] = encode(v.single_neighbor_step_m);
 n["single_neighbor_budget_m"] = encode(v.single_neighbor_budget_m);
 n["single_neighbor_max_age_s"] = encode(v.single_neighbor_max_age_s);
 n["observe_recovery_after_s"] = encode(v.observe_recovery_after_s);
 n["observe_recovery_lift_m"] = encode(v.observe_recovery_lift_m);
 n["observe_recovery_speed_m_s"] = encode(v.observe_recovery_speed_m_s);
 n["save_debug_images"] = encode(v.save_debug_images);
 n["debug_image_period_s"] = encode(v.debug_image_period_s);
 return n;
}
}
std::string effective_yaml(const MoveBoxConfig &cfg) {
 YAML::Emitter e;e.SetDoublePrecision(17);e<<encode(cfg);if(!e.good())throw std::runtime_error(e.GetLastError());return e.c_str();
}
}
