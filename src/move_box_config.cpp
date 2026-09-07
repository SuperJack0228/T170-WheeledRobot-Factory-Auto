#include "move_box_config.h"

#include "seg_pose_bridge.h"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <iostream>
#include <vector>

namespace
{

constexpr double kDeg2Rad = M_PI / 180.0;

Eigen::Matrix<double, 1, 6> pose6_from_deg_list(const std::array<double, 6> &v)
{
    Eigen::Matrix<double, 1, 6> pose;
    for (int i = 0; i < 3; ++i)
        pose(i) = v[static_cast<size_t>(i)];
    for (int i = 3; i < 6; ++i)
        pose(i) = v[static_cast<size_t>(i)] * kDeg2Rad;
    return pose;
}

bool read_pose6_deg(const YAML::Node &node, Eigen::Matrix<double, 1, 6> &out, std::string &err)
{
    if (!node || !node.IsSequence() || node.size() < 6)
    {
        err = "需要 6 个数 [x,y,z,rx,ry,rz](deg)";
        return false;
    }
    std::array<double, 6> v{};
    for (size_t i = 0; i < 6; ++i)
        v[i] = node[i].as<double>();
    out = pose6_from_deg_list(v);
    return true;
}

template <typename T>
void read_scalar(const YAML::Node &root, const char *key, T &out)
{
    if (root[key])
        out = root[key].as<T>();
}

void read_row_waist_x(const YAML::Node &place_node, MoveBoxPlaceConfig &place)
{
    const int row_count = std::max(1, static_cast<int>(place.row_x_bounds.size()));
    if (place.row_waist_x.size() < static_cast<size_t>(row_count))
        place.row_waist_x.resize(static_cast<size_t>(row_count), 0.0);

    if (const YAML::Node rw = place_node["row_waist_x"])
    {
        if (rw.IsSequence())
        {
            for (size_t i = 0; i < rw.size() && i < place.row_waist_x.size(); ++i)
                place.row_waist_x[i] = rw[i].as<double>();
        }
        else if (rw.IsMap())
        {
            for (int i = 0; i < row_count; ++i)
            {
                const std::string key = "row" + std::to_string(i + 1);
                if (rw[key])
                    place.row_waist_x[static_cast<size_t>(i)] = rw[key].as<double>();
            }
        }
    }

    // 兼容旧键：仅覆盖第 1 排
    if (place_node["first_row_waist_retreat_x"])
        place.row_waist_x[0] = place_node["first_row_waist_retreat_x"].as<double>();
}

void read_row_enabled(const YAML::Node &place_node, MoveBoxPlaceConfig &place)
{
    const int row_count = std::max(1, static_cast<int>(place.row_x_bounds.size()));
    place.row_enabled.assign(static_cast<size_t>(row_count), 1);

    if (const YAML::Node re = place_node["row_enabled"])
    {
        if (re.IsSequence())
        {
            for (size_t i = 0; i < re.size() && i < place.row_enabled.size(); ++i)
                place.row_enabled[i] = re[i].as<int>() != 0 ? 1 : 0;
        }
        else if (re.IsMap())
        {
            for (int i = 0; i < row_count; ++i)
            {
                const std::string key = "row" + std::to_string(i + 1);
                if (re[key])
                    place.row_enabled[static_cast<size_t>(i)] = re[key].as<int>() != 0 ? 1 : 0;
            }
        }
    }
}

int clamp_vision_algorithm_id(int v)
{
    if (v < -1)
        return -1;
    if (v > 1)
        return 1;
    return v;
}

void read_vision_detect(const YAML::Node &root, MoveBoxVisionDetectConfig &out)
{
    if (!root["vision_detect"])
        return;

    const YAML::Node n = root["vision_detect"];
    if (n["head_grasp"])
        out.head_grasp = clamp_vision_algorithm_id(n["head_grasp"].as<int>());
    if (n["right_hand_grasp"])
        out.right_hand_grasp = clamp_vision_algorithm_id(n["right_hand_grasp"].as<int>());
    if (n["left_hand_grasp"])
        out.left_hand_grasp = clamp_vision_algorithm_id(n["left_hand_grasp"].as<int>());
    if (n["place_holes"])
        out.place_holes = clamp_vision_algorithm_id(n["place_holes"].as<int>());
}

const char *vision_algorithm_label(int algorithm_id)
{
    switch (algorithm_id)
    {
    case 0:
        return "pnp";
    case 1:
        return "centroid";
    default:
        return "yaml";
    }
}

void read_place_hand_node(const YAML::Node &n, MoveBoxPlaceOffsetConfig &out)
{
    if (!n)
        return;
    read_scalar(n, "offset_x", out.offset_x);
    read_scalar(n, "offset_y", out.offset_y);
}

void read_place_row_hand_offsets(const YAML::Node &n, MoveBoxPlaceRowHandOffsets &out)
{
    if (const YAML::Node nl = n["left"])
        read_place_hand_node(nl, out.left);
    if (const YAML::Node nr = n["right"])
        read_place_hand_node(nr, out.right);
}

void fill_all_place_row_xy_offsets(MoveBoxPlaceConfig &place, const MoveBoxPlaceRowHandOffsets &src)
{
    for (auto &row : place.row_xy_offset)
        row = src;
    place.row6_xy_offset = src;
}

void read_row_xy_offsets(const YAML::Node &place_node, MoveBoxPlaceConfig &place)
{
    if (const YAML::Node rx = place_node["row_xy_offset"])
    {
        static const char *keys[] = {"row1", "row2", "row3", "row4", "row5"};
        for (int i = 0; i < 5; ++i)
        {
            if (const YAML::Node row = rx[keys[i]])
                read_place_row_hand_offsets(row, place.row_xy_offset[static_cast<size_t>(i)]);
        }
    }
    if (const YAML::Node r6 = place_node["row6_xy_offset"])
        read_place_row_hand_offsets(r6, place.row6_xy_offset);

    // 兼容旧版 place.left / place.right：未配 row_xy_offset 时五排+第六排共用
    if (!place_node["row_xy_offset"] && (place_node["left"] || place_node["right"]))
    {
        MoveBoxPlaceRowHandOffsets legacy;
        if (const YAML::Node nl = place_node["left"])
            read_place_hand_node(nl, legacy.left);
        if (const YAML::Node nr = place_node["right"])
            read_place_hand_node(nr, legacy.right);
        fill_all_place_row_xy_offsets(place, legacy);
    }
}

void read_row_x_bounds(const YAML::Node &place_node, MoveBoxPlaceConfig &place)
{
    if (!place_node["row_x_bounds"])
        return;

    const YAML::Node rb = place_node["row_x_bounds"];
    if (rb.IsSequence())
    {
        place.row_x_bounds.clear();
        for (size_t i = 0; i < rb.size(); ++i)
        {
            PlaceRowXBound b;
            if (rb[i].IsSequence() && rb[i].size() >= 2)
            {
                b.x_min = rb[i][0].as<double>();
                b.x_max = rb[i][1].as<double>();
            }
            else
            {
                read_scalar(rb[i], "x_min", b.x_min);
                read_scalar(rb[i], "x_max", b.x_max);
            }
            place.row_x_bounds.push_back(b);
        }
    }
}

} // namespace

MoveBoxConfig default_move_box_config()
{
    MoveBoxConfig cfg;
    cfg.standby.right = pose6_from_deg_list({0.45, -0.36, -0.15, -90, 0, 0});
    cfg.standby.left = pose6_from_deg_list({0.45, 0.36, -0.15, 90, 0, 0});
    cfg.waist.layer3_home = pose6_from_deg_list({-0.132502, 0.0, 0.622464, -90, -89.77, 180});
    {
        MoveBoxPlaceRowHandOffsets def_xy;
        def_xy.left.offset_x = 0.045;
        def_xy.left.offset_y = -0.04;
        def_xy.right.offset_x = 0.058;
        def_xy.right.offset_y = -0.015;
        fill_all_place_row_xy_offsets(cfg.place, def_xy);
    }
    cfg.place.fallback_right = pose6_from_deg_list({0.45, -0.36, -0.15, -90, 0, 0});
    cfg.place.fallback_left = pose6_from_deg_list({0.45, 0.36, -0.15, 90, 0, 0});
    cfg.place.row_waist_x = {-0.1, -0.1, -0.1, -0.1, -0.1, -0.1};
    cfg.place.row_enabled = {1, 1, 1, 1, 1, 1};
    cfg.place.row_place_from_front = 1;
    cfg.place.place_non_anchor_first = 1;
    cfg.place.detect_trials = 1;
    cfg.head_grasp.hand_detect_invalid_redo_max = 1;
    return cfg;
}

std::string default_move_box_config_path()
{
    return project_root_dir() + "/config/move_box_params.yaml";
}

bool load_move_box_config(const std::string &path, MoveBoxConfig &cfg, std::string &err)
{
    cfg = default_move_box_config();
    try
    {
        const YAML::Node root = YAML::LoadFile(path);

        if (const YAML::Node n = root["standby"])
        {
            if (n["right"])
                read_pose6_deg(n["right"], cfg.standby.right, err);
            if (n["left"])
                read_pose6_deg(n["left"], cfg.standby.left, err);
        }

        if (const YAML::Node n = root["head_grasp"])
        {
            read_scalar(n, "goal_z_base", cfg.head_grasp.goal_z_base);
            read_scalar(n, "goal_x_offset", cfg.head_grasp.goal_x_offset);
            read_scalar(n, "goal_z_extra", cfg.head_grasp.goal_z_extra);
            read_scalar(n, "hover_above_m", cfg.head_grasp.hover_above_m);
            read_scalar(n, "right_rx_deg", cfg.head_grasp.right_rx_deg);
            read_scalar(n, "right_ry_deg", cfg.head_grasp.right_ry_deg);
            read_scalar(n, "right_rz_deg", cfg.head_grasp.right_rz_deg);
            read_scalar(n, "left_rx_deg", cfg.head_grasp.left_rx_deg);
            read_scalar(n, "left_ry_deg", cfg.head_grasp.left_ry_deg);
            read_scalar(n, "left_rz_deg", cfg.head_grasp.left_rz_deg);
            read_scalar(n, "head_approach_z_descend", cfg.head_grasp.head_approach_z_descend);
            read_scalar(n, "hand_descend_z", cfg.head_grasp.hand_descend_z);
            if (const YAML::Node hg = n["hand_grasp"])
            {
                if (const YAML::Node nr = hg["right"])
                {
                    read_scalar(nr, "offset_x", cfg.head_grasp.hand_right.offset_x);
                    read_scalar(nr, "offset_y", cfg.head_grasp.hand_right.offset_y);
                }
                if (const YAML::Node nl = hg["left"])
                {
                    read_scalar(nl, "offset_x", cfg.head_grasp.hand_left.offset_x);
                    read_scalar(nl, "offset_y", cfg.head_grasp.hand_left.offset_y);
                }
            }
            read_scalar(n, "lift_after_grasp_z", cfg.head_grasp.lift_after_grasp_z);
            read_scalar(n, "hand_detect_invalid_redo_max", cfg.head_grasp.hand_detect_invalid_redo_max);
        }

        if (const YAML::Node n = root["place"])
        {
            read_row_xy_offsets(n, cfg.place);
            read_scalar(n, "z_raise", cfg.place.z_raise);
            read_scalar(n, "z_descend", cfg.place.z_descend);
            read_row_x_bounds(n, cfg.place);
            read_row_waist_x(n, cfg.place);
            read_row_enabled(n, cfg.place);
            read_scalar(n, "row_place_from_front", cfg.place.row_place_from_front);
            read_scalar(n, "place_non_anchor_first", cfg.place.place_non_anchor_first);
            read_scalar(n, "detect_trials", cfg.place.detect_trials);
            read_scalar(n, "place_pose_debug", cfg.place.place_pose_debug);
            if (const YAML::Node fb = n["fallback"])
            {
                if (fb["right"])
                    read_pose6_deg(fb["right"], cfg.place.fallback_right, err);
                if (fb["left"])
                    read_pose6_deg(fb["left"], cfg.place.fallback_left, err);
            }
            if (const YAML::Node rb = n["row6_bend"])
            {
                auto &b = cfg.place.row6_bend;
                read_scalar(rb, "enabled", b.enabled);
                read_scalar(rb, "row_index_0", b.row_index_0);
                read_scalar(rb, "pivot_z_below_base_m", b.pivot_z_below_base_m);
                read_scalar(rb, "waist_pitch_deg", b.waist_pitch_deg);
                read_scalar(rb, "shoulder_lift_deg", b.shoulder_lift_deg);
                read_scalar(rb, "right_rx_deg", b.right_rx_deg);
                read_scalar(rb, "right_ry_deg", b.right_ry_deg);
                read_scalar(rb, "right_rz_deg", b.right_rz_deg);
                read_scalar(rb, "left_rx_deg", b.left_rx_deg);
                read_scalar(rb, "left_ry_deg", b.left_ry_deg);
                read_scalar(rb, "left_rz_deg", b.left_rz_deg);
                read_scalar(rb, "speed_deg_per_s", b.speed_deg_per_s);
                read_scalar(rb, "smooth_dt_ms", b.smooth_dt_ms);
                read_scalar(rb, "waist_pitch_motor_can_id", b.waist_pitch_motor_can_id);
                read_scalar(rb, "right_shoulder_joint_index", b.right_shoulder_joint_index);
                read_scalar(rb, "left_shoulder_joint_index", b.left_shoulder_joint_index);
                read_scalar(rb, "z_raise", b.z_raise);
                read_scalar(rb, "z_descend", b.z_descend);
                read_scalar(rb, "retreat_y_right", b.retreat_y_right);
                read_scalar(rb, "retreat_y_left", b.retreat_y_left);
                read_scalar(rb, "retreat_x_delta_m", b.retreat_x_delta_m);
                read_scalar(rb, "retreat_z_delta_m", b.retreat_z_delta_m);
                read_scalar(rb, "retreat_right_rx_deg", b.retreat_right_rx_deg);
                read_scalar(rb, "retreat_right_ry_deg", b.retreat_right_ry_deg);
                read_scalar(rb, "retreat_right_rz_deg", b.retreat_right_rz_deg);
                read_scalar(rb, "retreat_left_rx_deg", b.retreat_left_rx_deg);
                read_scalar(rb, "retreat_left_ry_deg", b.retreat_left_ry_deg);
                read_scalar(rb, "retreat_left_rz_deg", b.retreat_left_rz_deg);
            }
        }

        if (const YAML::Node n = root["grasp_valid"])
        {
            read_scalar(n, "z_min", cfg.grasp_valid.z_min);
            read_scalar(n, "z_max", cfg.grasp_valid.z_max);
            read_scalar(n, "x_min", cfg.grasp_valid.x_min);
            read_scalar(n, "x_max", cfg.grasp_valid.x_max);
            read_scalar(n, "right_y_min", cfg.grasp_valid.right_y_min);
            read_scalar(n, "right_y_max", cfg.grasp_valid.right_y_max);
            read_scalar(n, "left_y_min", cfg.grasp_valid.left_y_min);
            read_scalar(n, "left_y_max", cfg.grasp_valid.left_y_max);
        }

        if (const YAML::Node n = root["waist"])
        {
            if (n["layer3_home"])
                read_pose6_deg(n["layer3_home"], cfg.waist.layer3_home, err);
            read_scalar(n, "stagger_step_x", cfg.waist.stagger_step_x);
            read_scalar(n, "stagger_max_steps", cfg.waist.stagger_max_steps);
            read_scalar(n, "place_advance_x", cfg.waist.place_advance_x);
            read_scalar(n, "move_settle_sec", cfg.waist.move_settle_sec);
            read_scalar(n, "place_ready_chassis_delay_sec", cfg.waist.place_ready_chassis_delay_sec);
            read_scalar(n, "place_ready_waist_delay_sec", cfg.waist.place_ready_waist_delay_sec);
            read_scalar(n, "x_min", cfg.waist.x_min);
            read_scalar(n, "x_max", cfg.waist.x_max);
            read_scalar(n, "z_min", cfg.waist.z_min);
            read_scalar(n, "z_max", cfg.waist.z_max);
            read_scalar(n, "grasp_object_z_min", cfg.waist.grasp_object_z_min);
            read_scalar(n, "grasp_lower_z", cfg.waist.grasp_lower_z);
        }

        if (const YAML::Node n = root["stagger"])
            read_scalar(n, "head_x_threshold", cfg.stagger.head_x_threshold);

        if (const YAML::Node n = root["grasp_zone"])
        {
            read_scalar(n, "y_side_split", cfg.grasp_zone.y_side_split);
            read_scalar(n, "y_max_abs", cfg.grasp_zone.y_max_abs);
            read_scalar(n, "x_min", cfg.grasp_zone.x_min);
            read_scalar(n, "x_max", cfg.grasp_zone.x_max);
            read_scalar(n, "edge_margin_frac", cfg.grasp_zone.edge_margin_frac);
            read_scalar(n, "cam_xy_over_z_max", cfg.grasp_zone.cam_xy_over_z_max);
        }

        if (const YAML::Node n = root["place_zone"])
        {
            read_scalar(n, "y_side_split", cfg.place_zone.y_side_split);
            read_scalar(n, "y_left", cfg.place_zone.y_left);
            read_scalar(n, "y_right", cfg.place_zone.y_right);
        }

        if (const YAML::Node n = root["aruco"])
        {
            read_scalar(n, "trials", cfg.aruco.trials);
            int slots = static_cast<int>(cfg.aruco.grid_slot_count);
            read_scalar(n, "grid_slot_count", slots);
            cfg.aruco.grid_slot_count = static_cast<size_t>(std::max(0, slots));
        }

        if (const YAML::Node n = root["cameras"])
        {
            read_scalar(n, "width", cfg.cameras.width);
            read_scalar(n, "height", cfg.cameras.height);
            read_scalar(n, "head_fps", cfg.cameras.head_fps);
            read_scalar(n, "hand_fps", cfg.cameras.hand_fps);
        }

        if (const YAML::Node n = root["battery"])
        {
            read_scalar(n, "low_threshold", cfg.battery.low_threshold);
            read_scalar(n, "full_threshold", cfg.battery.full_threshold);
        }

        read_vision_detect(root, cfg.vision_detect);

        if (const YAML::Node n = root["conveyor"])
        {
            read_scalar(n, "right_offset_m", cfg.conveyor.right_offset_m);
            read_scalar(n, "left_offset_m", cfg.conveyor.left_offset_m);
            read_scalar(n, "height_above_m", cfg.conveyor.height_above_m);
            read_scalar(n, "above_height_m", cfg.conveyor.above_height_m);
            read_scalar(n, "speed", cfg.conveyor.speed);
        }

        if (const YAML::Node n = root["ik"])
        {
            std::string method = "analytic";
            read_scalar(n, "method", method);
            if (method == "hybrid" || method == "mix")
                cfg.ik.method = MoveBoxIkMethod::Hybrid;
            else if (method == "numeric" || method == "num")
                cfg.ik.method = MoveBoxIkMethod::Numeric;
            else
                cfg.ik.method = MoveBoxIkMethod::Analytic;
            int from_cur = cfg.ik.j2_from_current ? 1 : 0;
            read_scalar(n, "j2_from_current", from_cur);
            cfg.ik.j2_from_current = from_cur != 0;
            read_scalar(n, "j2_right_deg", cfg.ik.j2_right_deg);
            read_scalar(n, "j2_left_deg", cfg.ik.j2_left_deg);
        }

        return true;
    }
    catch (const YAML::Exception &e)
    {
        err = std::string("YAML 解析失败: ") + e.what();
        return false;
    }
    catch (const std::exception &e)
    {
        err = std::string("读取配置失败: ") + e.what();
        return false;
    }
}

void print_move_box_config(const MoveBoxConfig &cfg)
{
    const auto print_pose = [](const char *tag, const Eigen::Matrix<double, 1, 6> &p) {
        std::cout << "[cfg] " << tag << " xyz=(" << p(0) << "," << p(1) << "," << p(2)
                  << ") m rpy=(" << (p(3) * 180.0 / M_PI) << "," << (p(4) * 180.0 / M_PI) << ","
                  << (p(5) * 180.0 / M_PI) << ") deg\n";
    };
    std::cout << "[cfg] 已加载 move_box_params\n";
    print_pose("standby.right", cfg.standby.right);
    print_pose("standby.left", cfg.standby.left);
    print_pose("waist.layer3_home", cfg.waist.layer3_home);
    std::cout << "[cfg] head_grasp hover=" << cfg.head_grasp.hover_above_m
              << " z_base=" << cfg.head_grasp.goal_z_base
              << " x_off=" << cfg.head_grasp.goal_x_offset
              << " z_extra=" << cfg.head_grasp.goal_z_extra
              << " head_desc=" << cfg.head_grasp.head_approach_z_descend
              << " hand_desc=" << cfg.head_grasp.hand_descend_z
              << " hand_xy_r=(" << cfg.head_grasp.hand_right.offset_x << ","
              << cfg.head_grasp.hand_right.offset_y << ")"
              << " hand_xy_l=(" << cfg.head_grasp.hand_left.offset_x << ","
              << cfg.head_grasp.hand_left.offset_y << ")"
              << " lift=" << cfg.head_grasp.lift_after_grasp_z
              << " rpy_r=(" << cfg.head_grasp.right_rx_deg << "," << cfg.head_grasp.right_ry_deg
              << "," << cfg.head_grasp.right_rz_deg << ") rpy_l=(" << cfg.head_grasp.left_rx_deg
              << "," << cfg.head_grasp.left_ry_deg << "," << cfg.head_grasp.left_rz_deg << ")\n";
    std::cout << "[cfg] place row_xy_offset:";
    for (size_t i = 0; i < cfg.place.row_xy_offset.size(); ++i)
    {
        const auto &row = cfg.place.row_xy_offset[i];
        std::cout << " r" << (i + 1) << " L=(" << row.left.offset_x << "," << row.left.offset_y
                  << ") R=(" << row.right.offset_x << "," << row.right.offset_y << ")";
    }
    const auto &r6 = cfg.place.row6_xy_offset;
    std::cout << " row6 L=(" << r6.left.offset_x << "," << r6.left.offset_y << ") R=("
              << r6.right.offset_x << "," << r6.right.offset_y << ")"
              << " z_raise=" << cfg.place.z_raise << " z_descend=" << cfg.place.z_descend << "\n";
    print_pose("place.fallback.right", cfg.place.fallback_right);
    print_pose("place.fallback.left", cfg.place.fallback_left);
    std::cout << "[cfg] place row_waist_x:";
    for (size_t i = 0; i < cfg.place.row_waist_x.size(); ++i)
        std::cout << " r" << (i + 1) << "=" << cfg.place.row_waist_x[i]
                  << "(arm+=" << -cfg.place.row_waist_x[i] << ")";
    std::cout << "\n";
    std::cout << "[cfg] place row_enabled:";
    for (size_t i = 0; i < cfg.place.row_enabled.size(); ++i)
        std::cout << " r" << (i + 1) << "=" << cfg.place.row_enabled[i];
    std::cout << " from_front=" << cfg.place.row_place_from_front
              << " place_non_anchor_first=" << cfg.place.place_non_anchor_first
              << " detect_trials=" << cfg.place.detect_trials
              << " place_pose_debug=" << (cfg.place.place_pose_debug ? "on" : "off") << "\n";
    const auto &rb = cfg.place.row6_bend;
    std::cout << "[cfg] place.row6_bend enabled=" << rb.enabled
              << " row=" << (rb.row_index_0 + 1) << " pivot_z=" << rb.pivot_z_below_base_m
              << " waist_pitch=" << rb.waist_pitch_deg << " shoulder=" << rb.shoulder_lift_deg
              << " z_raise=" << rb.z_raise << " z_descend=" << rb.z_descend
              << " ry=" << rb.right_ry_deg << "/" << rb.left_ry_deg
              << " retreat_y=" << rb.retreat_y_right << "/" << rb.retreat_y_left << "\n";
    std::cout << "[cfg] place row_x_bounds:";
    for (size_t i = 0; i < cfg.place.row_x_bounds.size(); ++i)
        std::cout << " r" << (i + 1) << "=[" << cfg.place.row_x_bounds[i].x_min << ","
                  << cfg.place.row_x_bounds[i].x_max << ")";

    std::cout << "[cfg] hand_detect invalid_redo_max=" << cfg.head_grasp.hand_detect_invalid_redo_max << "\n";
    std::cout << "[cfg] stagger x>" << cfg.stagger.head_x_threshold
              << " waist_step=" << cfg.waist.stagger_step_x
              << " place_advance=" << cfg.waist.place_advance_x
              << " place_ready_delay(chassis/waist)="
              << cfg.waist.place_ready_chassis_delay_sec << "s/"
              << cfg.waist.place_ready_waist_delay_sec << "s"
              << " layer3 x=[" << cfg.waist.x_min << "," << cfg.waist.x_max << "]"
              << " z=[" << cfg.waist.z_min << "," << cfg.waist.z_max << "]"
              << " grasp_z<" << cfg.waist.grasp_object_z_min
              << " → lower " << cfg.waist.grasp_lower_z << "m\n";
    std::cout << "[cfg] grasp_zone side_split=" << cfg.grasp_zone.y_side_split
              << " (中间±" << cfg.grasp_zone.y_side_split << " y=0分左右)"
              << " y_max=" << cfg.grasp_zone.y_max_abs
              << " x=[" << cfg.grasp_zone.x_min << "," << cfg.grasp_zone.x_max << "]"
              << " edge=" << cfg.grasp_zone.edge_margin_frac
              << " cam_xy/z<" << cfg.grasp_zone.cam_xy_over_z_max << "\n";
    std::cout << "[cfg] place_zone side_split=" << cfg.place_zone.y_side_split
              << " middle=[" << cfg.place_zone.y_right << "," << cfg.place_zone.y_left << "]\n";
    const auto &vd = cfg.vision_detect;
    std::cout << "[cfg] vision_detect head=" << vd.head_grasp << "(" << vision_algorithm_label(vd.head_grasp)
              << ") right_hand=" << vd.right_hand_grasp << "(" << vision_algorithm_label(vd.right_hand_grasp)
              << ") left_hand=" << vd.left_hand_grasp << "(" << vision_algorithm_label(vd.left_hand_grasp)
              << ") place_holes=" << vd.place_holes << "(" << vision_algorithm_label(vd.place_holes) << ")\n";
    std::cout << "[cfg] cameras " << cfg.cameras.width << "x" << cfg.cameras.height
              << " head_fps=" << cfg.cameras.head_fps
              << " hand_fps=" << cfg.cameras.hand_fps << "\n";
    std::cout << "[cfg] battery low<" << cfg.battery.low_threshold
              << " full>" << cfg.battery.full_threshold << "\n";
    std::cout << "[cfg] conveyor right_off=" << cfg.conveyor.right_offset_m
              << " left_off=" << cfg.conveyor.left_offset_m
              << " height=" << cfg.conveyor.height_above_m
              << " above=" << cfg.conveyor.above_height_m
              << " speed=" << cfg.conveyor.speed << "\n";
    const char *ik_name =
        cfg.ik.method == MoveBoxIkMethod::Hybrid ? "hybrid"
        : cfg.ik.method == MoveBoxIkMethod::Numeric ? "numeric"
                                                    : "analytic";
    std::cout << "[cfg] ik method=" << ik_name
              << " j2_from_current=" << cfg.ik.j2_from_current
              << " j2_right=" << cfg.ik.j2_right_deg
              << " j2_left=" << cfg.ik.j2_left_deg << " deg\n";
}

const MoveBoxPlaceOffsetConfig &place_hand_xy_offset(int row_0, bool is_right)
{
    if (row_0 == 5)
        return is_right ? g_move_cfg.place.row6_xy_offset.right : g_move_cfg.place.row6_xy_offset.left;
    const int idx = (row_0 >= 0 && row_0 < 5) ? row_0 : 0;
    return is_right ? g_move_cfg.place.row_xy_offset[static_cast<size_t>(idx)].right
                    : g_move_cfg.place.row_xy_offset[static_cast<size_t>(idx)].left;
}
