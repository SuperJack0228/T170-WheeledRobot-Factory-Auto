#include "move_box_config.h"

#include "seg_pose_bridge.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
bool g_have_live_tray_origin_y = false;
double g_live_tray_origin_y = 0.0;
} // namespace

namespace
{

constexpr double kDegToRad = M_PI / 180.0;

template <typename T>
void read_value(const YAML::Node &node, const char *key, T &out)
{
    if (node && node[key])
        out = node[key].as<T>();
}

bool read_pose(const YAML::Node &node, Eigen::Matrix<double, 1, 6> &out)
{
    if (!node || !node.IsSequence() || node.size() != 6)
        return false;
    for (int i = 0; i < 3; ++i)
        out(i) = node[static_cast<size_t>(i)].as<double>();
    for (int i = 3; i < 6; ++i)
        out(i) = node[static_cast<size_t>(i)].as<double>() * kDegToRad;
    return true;
}

void read_hand_offset(const YAML::Node &node, HandXyOffsetConfig &out)
{
    read_value(node, "offset_x", out.offset_x);
    read_value(node, "offset_y", out.offset_y);
}

int clamp_algorithm_id(int value)
{
    return (value == 0 || value == 1) ? value : -1;
}

void load_conveyor_station(const YAML::Node &node, ConveyorStationConfig &dst, const char *name)
{
    if (!node)
        return;
    read_value(node, "waist_x", dst.waist_x);
    read_value(node, "waist_z", dst.waist_z);
    read_value(node, "head_yaw_deg", dst.head_yaw_deg);
    read_value(node, "head_pitch_deg", dst.head_pitch_deg);
    read_value(node, "head_roll_deg", dst.head_roll_deg);
    if (node["tcp"])
    {
        if (!read_pose(node["tcp"]["right"], dst.tcp.right) ||
            !read_pose(node["tcp"]["left"], dst.tcp.left))
            throw std::runtime_error(std::string(name) + ".tcp.left/right 必须各有 6 个数");
    }
    if (node["place_tcp"])
    {
        if (!read_pose(node["place_tcp"]["right"], dst.place_tcp.right) ||
            !read_pose(node["place_tcp"]["left"], dst.place_tcp.left))
            throw std::runtime_error(std::string(name) + ".place_tcp.left/right 必须各有 6 个数");
    }
    else if (node["tcp"])
        dst.place_tcp = dst.tcp;
    if (node["offset"])
    {
        read_value(node["offset"]["right"], "x", dst.offset_right.x);
        read_value(node["offset"]["right"], "y", dst.offset_right.y);
        read_value(node["offset"]["right"], "z", dst.offset_right.z);
        read_value(node["offset"]["left"], "x", dst.offset_left.x);
        read_value(node["offset"]["left"], "y", dst.offset_left.y);
        read_value(node["offset"]["left"], "z", dst.offset_left.z);
    }
    read_value(node, "prefer_belt", dst.prefer_belt);
    auto read_grasp_rpy = [](const YAML::Node &rpy, GraspRpyDeg &out) -> bool {
        if (!rpy || !rpy.IsSequence() || rpy.size() < 3)
            return false;
        out.rx = rpy[0].as<double>();
        out.ry = rpy[1].as<double>();
        out.rz = rpy[2].as<double>();
        return true;
    };
    if (node["grasp_rpy_deg"])
    {
        if (node["grasp_rpy_deg"]["left"] &&
            !read_grasp_rpy(node["grasp_rpy_deg"]["left"], dst.grasp_rpy_left))
            throw std::runtime_error(std::string(name) + ".grasp_rpy_deg.left 必须是 3 个数");
        if (node["grasp_rpy_deg"]["right"] &&
            !read_grasp_rpy(node["grasp_rpy_deg"]["right"], dst.grasp_rpy_right))
            throw std::runtime_error(std::string(name) + ".grasp_rpy_deg.right 必须是 3 个数");
    }
    if (node["grasp_offset"])
    {
        const YAML::Node go = node["grasp_offset"];
        if (go["left"] || go["right"])
        {
            read_value(go["left"], "x", dst.grasp_offset_left.x);
            read_value(go["left"], "y", dst.grasp_offset_left.y);
            read_value(go["left"], "z", dst.grasp_offset_left.z);
            read_value(go["right"], "x", dst.grasp_offset_right.x);
            read_value(go["right"], "y", dst.grasp_offset_right.y);
            read_value(go["right"], "z", dst.grasp_offset_right.z);
        }
        else
        {
            read_value(go, "x", dst.grasp_offset_left.x);
            read_value(go, "y", dst.grasp_offset_left.y);
            read_value(go, "z", dst.grasp_offset_left.z);
            dst.grasp_offset_right = dst.grasp_offset_left;
        }
    }
    if (node["grasp_tcp"])
    {
        if (!read_pose(node["grasp_tcp"]["right"], dst.grasp_tcp.right) ||
            !read_pose(node["grasp_tcp"]["left"], dst.grasp_tcp.left))
            throw std::runtime_error(std::string(name) + ".grasp_tcp.left/right 必须各有 6 个数");
    }
    else if (node["tcp"])
        dst.grasp_tcp = dst.tcp;
    read_value(node, "grasp_hover_above_m", dst.grasp_hover_above_m);
    read_value(node, "place_hover_above_m", dst.place_hover_above_m);
    read_value(node, "grasp_waist_yaw_deg", dst.grasp_waist_yaw_deg);
    read_value(node, "grasp_ready_close_ratio", dst.grasp_ready_close_ratio);
    read_value(node, "grasp_yolo_enable", dst.grasp_yolo_enable);
    read_value(node, "grasp_yolo_class_id", dst.grasp_yolo_class_id);
    read_value(node, "use_fixed_origin_z", dst.use_fixed_origin_z);
    read_value(node, "fixed_origin_z_m", dst.fixed_origin_z_m);
}

void print_conveyor_station(const ConveyorStationConfig &c, const char *name)
{
    std::cout << std::fixed << std::setprecision(4)
              << "[cfg] " << name << " station x=" << c.waist_x
              << " z=" << c.waist_z
              << "m head yaw=" << c.head_yaw_deg
              << " pitch=" << c.head_pitch_deg
              << " roll=" << c.head_roll_deg << " deg\n"
              << "[cfg] " << name << " tcp right=" << c.tcp.right << "\n"
              << "[cfg] " << name << " tcp left=" << c.tcp.left << "\n"
              << "[cfg] " << name << " grasp_tcp right=" << c.grasp_tcp.right << "\n"
              << "[cfg] " << name << " grasp_tcp left=" << c.grasp_tcp.left << "\n"
              << "[cfg] " << name << " place_tcp right=" << c.place_tcp.right << "\n"
              << "[cfg] " << name << " place_tcp left=" << c.place_tcp.left << "\n"
              << "[cfg] " << name << " offset right=(" << c.offset_right.x
              << "," << c.offset_right.y << "," << c.offset_right.z
              << ") left=(" << c.offset_left.x << ","
              << c.offset_left.y << "," << c.offset_left.z
              << ") prefer=" << c.prefer_belt << "\n"
              << "[cfg] " << name << " grasp_rpy_deg 右=(" << c.grasp_rpy_right.rx
              << "," << c.grasp_rpy_right.ry << "," << c.grasp_rpy_right.rz
              << ") 左=(" << c.grasp_rpy_left.rx << ","
              << c.grasp_rpy_left.ry << "," << c.grasp_rpy_left.rz
              << ") offset_识别系 左=(" << c.grasp_offset_left.x << ","
              << c.grasp_offset_left.y << "," << c.grasp_offset_left.z
              << ") 右=(" << c.grasp_offset_right.x << ","
              << c.grasp_offset_right.y << "," << c.grasp_offset_right.z
              << ") hover=" << c.grasp_hover_above_m
              << "m yaw_left=" << c.grasp_waist_yaw_deg
              << "deg ready_grip=" << c.grasp_ready_close_ratio
              << " yolo=" << (c.grasp_yolo_enable ? "on" : "off")
              << " class=" << c.grasp_yolo_class_id
              << " fixed_z=" << (c.use_fixed_origin_z ? "on" : "off")
              << " " << c.fixed_origin_z_m << "m\n";
}

} // namespace

MoveBoxConfig default_move_box_config()
{
    MoveBoxConfig cfg;
    cfg.standby.right << 0.45, -0.30, -0.25, 0.0, 45.0 * kDegToRad, 40.0 * kDegToRad;
    cfg.standby.left << 0.45, 0.30, -0.25, 0.0, 45.0 * kDegToRad, -40.0 * kDegToRad;
    cfg.home_tcp.right.setZero();
    cfg.home_tcp.left.setZero();
    cfg.waist.layer3_home << 0.0, 0.0, 0.55, 0.0, 0.0, 0.0;
    for (int g = 0; g < 3; ++g)
    {
        cfg.head_grasp.left_row_rpy_deg[g] = {0.0, 60.0, -45.0};
        cfg.head_grasp.right_row_rpy_deg[g] = {0.0, 60.0, 45.0};
    }
    cfg.head_grasp.left_row6_rpy_deg = {0.0, 30.0, -20.0};
    cfg.head_grasp.right_row6_rpy_deg = {0.0, 30.0, 20.0};
    for (int g = 0; g < 3; ++g)
    {
        cfg.tray2_place.left_row_rpy_deg[g] = cfg.head_grasp.left_row_rpy_deg[g];
        cfg.tray2_place.right_row_rpy_deg[g] = cfg.head_grasp.right_row_rpy_deg[g];
    }
    cfg.tray2_place.left_row6_rpy_deg = cfg.head_grasp.left_row6_rpy_deg;
    cfg.tray2_place.right_row6_rpy_deg = cfg.head_grasp.right_row6_rpy_deg;
    cfg.head_grasp.goal_y_offset = 0.012;
    cfg.head_grasp.hand_left.offset_x = -0.01;
    cfg.head_grasp.hand_left.offset_y = -0.02;
    cfg.head_grasp.hand_right.offset_x = -0.01;
    cfg.head_grasp.hand_right.offset_y = 0.02;
    cfg.conveyor.tcp.right << 0.45, -0.30, -0.25, 0.0, 45.0 * kDegToRad, 40.0 * kDegToRad;
    cfg.conveyor.tcp.left << 0.45, 0.30, -0.25, 0.0, 45.0 * kDegToRad, -40.0 * kDegToRad;
    cfg.conveyor.place_tcp = cfg.conveyor.tcp;
    cfg.conveyor.grasp_rpy_left = {0.0, 45.0, -30.0};
    cfg.conveyor.grasp_rpy_right = {0.0, 45.0, 30.0};
    cfg.conveyor.grasp_offset_left = {0.00, 0.12, 0.00};
    cfg.conveyor.grasp_offset_right = {0.00, 0.12, 0.00};
    cfg.conveyor.grasp_tcp = cfg.conveyor.tcp;
    cfg.conveyor.offset_right = {-0.04, 0.00, 0.08};
    cfg.conveyor.offset_left = {-0.04, 0.00, 0.08};
    cfg.conveyor.grasp_hover_above_m = 0.06;
    cfg.conveyor.place_hover_above_m = 0.06;
    cfg.conveyor.grasp_waist_yaw_deg = 0.0;
    cfg.conveyor.grasp_ready_close_ratio = 0.30;
    cfg.conveyor.grasp_yolo_enable = false;
    cfg.conveyor.grasp_yolo_class_id = 1;
    cfg.conveyor2 = cfg.conveyor;
    cfg.conveyor2.grasp_yolo_class_id = 2;
    return cfg;
}

std::string default_move_box_config_path()
{
    return project_root_dir() + "/config/move_box_params.yaml";
}

bool load_move_box_config(const std::string &path, MoveBoxConfig &cfg, std::string &err)
{
    cfg = default_move_box_config();
    err.clear();
    try
    {
        const YAML::Node root = YAML::LoadFile(path);
        if (!read_pose(root["standby"]["right"], cfg.standby.right) ||
            !read_pose(root["standby"]["left"], cfg.standby.left))
            throw std::runtime_error("standby.left/right 必须各有 6 个数");
        if (root["home_tcp"])
        {
            if (!read_pose(root["home_tcp"]["right"], cfg.home_tcp.right) ||
                !read_pose(root["home_tcp"]["left"], cfg.home_tcp.left))
                throw std::runtime_error("home_tcp.left/right 必须各有 6 个数");
        }

        const YAML::Node tray = root["tray"];
        read_value(tray, "z_ref_m", cfg.tray.z_ref_m);
        read_value(tray, "z_ref_waist_z_m", cfg.tray.z_ref_waist_z_m);
        read_value(tray, "part_above_tray_m", cfg.tray.part_above_tray_m);
        read_value(tray, "z_refine_max_m", cfg.tray.z_refine_max_m);
        read_value(tray, "fuse_frames", cfg.tray.fuse_frames);

        const YAML::Node head = root["head"];
        read_value(head, "yaw_deg", cfg.head.yaw_deg);
        read_value(head, "pitch_deg", cfg.head.pitch_deg);
        read_value(head, "far_pitch_deg", cfg.head.far_pitch_deg);
        read_value(head, "roll_deg", cfg.head.roll_deg);
        read_value(head, "speed_deg_s", cfg.head.speed_deg_s);
        if (cfg.head.speed_deg_s <= 1e-6)
            throw std::runtime_error("head.speed_deg_s 必须为正");

        const YAML::Node hg = root["head_grasp"];
        read_value(hg, "hover_above_m", cfg.head_grasp.hover_above_m);
        read_value(hg, "goal_z_base", cfg.head_grasp.goal_z_base);
        read_value(hg, "goal_x_offset", cfg.head_grasp.goal_x_offset);
        read_value(hg, "goal_y_offset", cfg.head_grasp.goal_y_offset);
        read_value(hg, "goal_z_extra", cfg.head_grasp.goal_z_extra);
        read_value(hg, "right_rx_deg", cfg.head_grasp.right_rx_deg);
        read_value(hg, "right_ry_deg", cfg.head_grasp.right_ry_deg);
        read_value(hg, "right_rz_deg", cfg.head_grasp.right_rz_deg);
        read_value(hg, "left_rx_deg", cfg.head_grasp.left_rx_deg);
        read_value(hg, "left_ry_deg", cfg.head_grasp.left_ry_deg);
        read_value(hg, "left_rz_deg", cfg.head_grasp.left_rz_deg);
        read_value(hg, "use_bezier_grasp", cfg.head_grasp.use_bezier_grasp);
        for (int g = 0; g < 3; ++g)
        {
            cfg.head_grasp.left_row_rpy_deg[g] = {
                cfg.head_grasp.left_rx_deg,
                cfg.head_grasp.left_ry_deg,
                cfg.head_grasp.left_rz_deg};
            cfg.head_grasp.right_row_rpy_deg[g] = {
                cfg.head_grasp.right_rx_deg,
                cfg.head_grasp.right_ry_deg,
                cfg.head_grasp.right_rz_deg};
        }
        auto read_row_rpy_list = [](const YAML::Node &node, GraspRpyDeg out[3]) {
            if (!node || !node.IsSequence())
                return;
            const int n = std::min(3, static_cast<int>(node.size()));
            for (int g = 0; g < n; ++g)
            {
                const YAML::Node item = node[static_cast<size_t>(g)];
                if (!item || !item.IsSequence() || item.size() != 3)
                    continue;
                out[g].rx = item[0].as<double>();
                out[g].ry = item[1].as<double>();
                out[g].rz = item[2].as<double>();
            }
        };
        read_row_rpy_list(hg["row_grasp_rpy_deg"]["left"], cfg.head_grasp.left_row_rpy_deg);
        if (hg["row_grasp_rpy_deg"]["right"] && hg["row_grasp_rpy_deg"]["right"].IsSequence())
        {
            read_row_rpy_list(hg["row_grasp_rpy_deg"]["right"], cfg.head_grasp.right_row_rpy_deg);
        }
        else
        {
            for (int g = 0; g < 3; ++g)
            {
                cfg.head_grasp.right_row_rpy_deg[g] = cfg.head_grasp.left_row_rpy_deg[g];
                cfg.head_grasp.right_row_rpy_deg[g].rz = -cfg.head_grasp.left_row_rpy_deg[g].rz;
            }
            std::cout << "[cfg] 未读到 row_grasp_rpy_deg.right，用左手 rz 取反\n";
        }
        cfg.head_grasp.left_row6_rpy_deg = cfg.head_grasp.left_row_rpy_deg[0];
        cfg.head_grasp.right_row6_rpy_deg = cfg.head_grasp.right_row_rpy_deg[0];
        auto read_one_rpy = [](const YAML::Node &node, GraspRpyDeg &out) -> bool {
            YAML::Node seq = node;
            if (node && node.IsMap() && node["left"] && node["left"].IsSequence())
                seq = node["left"];
            if (!seq || !seq.IsSequence() || seq.size() < 3)
                return false;
            out.rx = seq[0].as<double>();
            out.ry = seq[1].as<double>();
            out.rz = seq[2].as<double>();
            return true;
        };
        if (!read_one_rpy(hg["row1_rpy_deg"]["left"], cfg.head_grasp.left_row6_rpy_deg) &&
            !read_one_rpy(hg["row1_rpy_deg"], cfg.head_grasp.left_row6_rpy_deg) &&
            !read_one_rpy(hg["row6_rpy_deg"]["left"], cfg.head_grasp.left_row6_rpy_deg) &&
            !read_one_rpy(hg["row6_rpy_deg"], cfg.head_grasp.left_row6_rpy_deg))
        {
            std::cerr << "[cfg] 未读到 row1_rpy_deg.left，最远行暂用 ready1 "
                      << cfg.head_grasp.left_row6_rpy_deg.rx << ","
                      << cfg.head_grasp.left_row6_rpy_deg.ry << ","
                      << cfg.head_grasp.left_row6_rpy_deg.rz << "\n";
        }
        else
        {
            std::cout << std::fixed << std::setprecision(1)
                      << "[cfg] 已加载 最远行 row1_rpy_deg.left=("
                      << cfg.head_grasp.left_row6_rpy_deg.rx << ","
                      << cfg.head_grasp.left_row6_rpy_deg.ry << ","
                      << cfg.head_grasp.left_row6_rpy_deg.rz << ")\n";
        }
        if (read_one_rpy(hg["row1_rpy_deg"]["right"], cfg.head_grasp.right_row6_rpy_deg) ||
            read_one_rpy(hg["row6_rpy_deg"]["right"], cfg.head_grasp.right_row6_rpy_deg))
        {
            std::cout << std::fixed << std::setprecision(1)
                      << "[cfg] 已加载 最远行 row1_rpy_deg.right=("
                      << cfg.head_grasp.right_row6_rpy_deg.rx << ","
                      << cfg.head_grasp.right_row6_rpy_deg.ry << ","
                      << cfg.head_grasp.right_row6_rpy_deg.rz << ")\n";
        }
        else
        {
            cfg.head_grasp.right_row6_rpy_deg = cfg.head_grasp.left_row6_rpy_deg;
            cfg.head_grasp.right_row6_rpy_deg.rz = -cfg.head_grasp.left_row6_rpy_deg.rz;
            std::cout << std::fixed << std::setprecision(1)
                      << "[cfg] 未读到 row1_rpy_deg.right，用左手 rz 取反 ("
                      << cfg.head_grasp.right_row6_rpy_deg.rx << ","
                      << cfg.head_grasp.right_row6_rpy_deg.ry << ","
                      << cfg.head_grasp.right_row6_rpy_deg.rz << ")\n";
        }
        for (int g = 0; g < 3; ++g)
        {
            cfg.tray2_place.left_row_rpy_deg[g] = cfg.head_grasp.left_row_rpy_deg[g];
            cfg.tray2_place.right_row_rpy_deg[g] = cfg.head_grasp.right_row_rpy_deg[g];
        }
        cfg.tray2_place.left_row6_rpy_deg = cfg.head_grasp.left_row6_rpy_deg;
        cfg.tray2_place.right_row6_rpy_deg = cfg.head_grasp.right_row6_rpy_deg;
        cfg.tray2_place.goal_x_offset = cfg.head_grasp.goal_x_offset;
        cfg.tray2_place.goal_y_offset = cfg.head_grasp.goal_y_offset;
        cfg.tray2_place.grasp_z_offset_m = cfg.head_grasp.grasp_z_offset_m;
        cfg.tray2_place.right_goal_x_offset = cfg.head_grasp.right_goal_x_offset;
        cfg.tray2_place.right_goal_y_offset = cfg.head_grasp.right_goal_y_offset;
        cfg.tray2_place.right_grasp_z_offset_m = cfg.head_grasp.right_grasp_z_offset_m;
        cfg.tray2_place.nearest_row_z_offset_m = cfg.head_grasp.nearest_row_grasp_z_offset_m;
        const YAML::Node t2 = root["tray2_place"];
        if (t2)
        {
            read_row_rpy_list(t2["row_place_rpy_deg"]["left"], cfg.tray2_place.left_row_rpy_deg);
            if (t2["row_place_rpy_deg"]["right"] && t2["row_place_rpy_deg"]["right"].IsSequence())
                read_row_rpy_list(t2["row_place_rpy_deg"]["right"], cfg.tray2_place.right_row_rpy_deg);
            else
            {
                for (int g = 0; g < 3; ++g)
                {
                    cfg.tray2_place.right_row_rpy_deg[g] = cfg.tray2_place.left_row_rpy_deg[g];
                    cfg.tray2_place.right_row_rpy_deg[g].rz = -cfg.tray2_place.left_row_rpy_deg[g].rz;
                }
            }
            if (read_one_rpy(t2["row1_rpy_deg"]["left"], cfg.tray2_place.left_row6_rpy_deg) ||
                read_one_rpy(t2["row1_rpy_deg"], cfg.tray2_place.left_row6_rpy_deg))
            {
                std::cout << std::fixed << std::setprecision(1)
                          << "[cfg] 已加载 tray2 最远行 row1_rpy_deg.left=("
                          << cfg.tray2_place.left_row6_rpy_deg.rx << ","
                          << cfg.tray2_place.left_row6_rpy_deg.ry << ","
                          << cfg.tray2_place.left_row6_rpy_deg.rz << ")\n";
            }
            if (read_one_rpy(t2["row1_rpy_deg"]["right"], cfg.tray2_place.right_row6_rpy_deg))
            {
                std::cout << std::fixed << std::setprecision(1)
                          << "[cfg] 已加载 tray2 最远行 row1_rpy_deg.right=("
                          << cfg.tray2_place.right_row6_rpy_deg.rx << ","
                          << cfg.tray2_place.right_row6_rpy_deg.ry << ","
                          << cfg.tray2_place.right_row6_rpy_deg.rz << ")\n";
            }
            else
            {
                cfg.tray2_place.right_row6_rpy_deg = cfg.tray2_place.left_row6_rpy_deg;
                cfg.tray2_place.right_row6_rpy_deg.rz = -cfg.tray2_place.left_row6_rpy_deg.rz;
            }
            if (t2["offset"])
            {
                const YAML::Node off = t2["offset"];
                read_value(off["left"], "x", cfg.tray2_place.goal_x_offset);
                read_value(off["left"], "y", cfg.tray2_place.goal_y_offset);
                read_value(off["left"], "z", cfg.tray2_place.grasp_z_offset_m);
                read_value(off["right"], "x", cfg.tray2_place.right_goal_x_offset);
                read_value(off["right"], "y", cfg.tray2_place.right_goal_y_offset);
                read_value(off["right"], "z", cfg.tray2_place.right_grasp_z_offset_m);
            }
            read_value(t2, "nearest_row_z_offset_m", cfg.tray2_place.nearest_row_z_offset_m);
        }
        read_value(hg, "adaptive_pitch_down_deg", cfg.head_grasp.adaptive_pitch_down_deg);
        read_value(hg, "adaptive_yaw_max_deg", cfg.head_grasp.adaptive_yaw_max_deg);
        read_value(hg, "adaptive_min_horiz_m", cfg.head_grasp.adaptive_min_horiz_m);
        read_value(hg, "adaptive_approach_xyz_tol_m", cfg.head_grasp.adaptive_approach_xyz_tol_m);
        read_value(hg, "adaptive_approach_rpy_tol_deg", cfg.head_grasp.adaptive_approach_rpy_tol_deg);
        read_value(hg, "head_approach_z_descend", cfg.head_grasp.head_approach_z_descend);
        read_value(hg, "hand_descend_z", cfg.head_grasp.hand_descend_z);
        read_value(hg, "grasp_z_offset_m", cfg.head_grasp.grasp_z_offset_m);
        cfg.head_grasp.right_goal_x_offset = cfg.head_grasp.goal_x_offset;
        cfg.head_grasp.right_goal_y_offset = -cfg.head_grasp.goal_y_offset;
        cfg.head_grasp.right_grasp_z_offset_m = cfg.head_grasp.grasp_z_offset_m;
        read_value(hg, "right_goal_x_offset", cfg.head_grasp.right_goal_x_offset);
        read_value(hg, "right_goal_y_offset", cfg.head_grasp.right_goal_y_offset);
        read_value(hg, "right_grasp_z_offset_m", cfg.head_grasp.right_grasp_z_offset_m);
        read_value(hg, "nearest_row_grasp_z_offset_m", cfg.head_grasp.nearest_row_grasp_z_offset_m);
        read_value(hg, "use_hand_camera", cfg.head_grasp.use_hand_camera);
        read_value(hg, "hand_match_xy_max_m", cfg.head_grasp.hand_match_xy_max_m);
        read_value(hg, "hand_xy_refine_max_m", cfg.head_grasp.hand_xy_refine_max_m);
        read_value(hg, "lift_after_grasp_z", cfg.head_grasp.lift_after_grasp_z);
        read_value(hg, "bezier_guide_height_m", cfg.head_grasp.bezier_guide_height_m);
        read_value(hg, "bezier_ab_gap_m", cfg.head_grasp.bezier_ab_gap_m);
        read_value(hg, "bezier_vel_m_s", cfg.head_grasp.bezier_vel_m_s);
        read_value(hg, "bezier_orient_finish_ratio", cfg.head_grasp.bezier_orient_finish_ratio);
        read_value(hg, "bezier_endpoint_xyz_tol_m", cfg.head_grasp.bezier_endpoint_xyz_tol_m);
        read_value(hg, "bezier_endpoint_rpy_tol_deg", cfg.head_grasp.bezier_endpoint_rpy_tol_deg);
        read_value(hg, "approach_vel_m_s", cfg.head_grasp.approach_vel_m_s);
        read_value(hg, "descend_vel_m_s", cfg.head_grasp.descend_vel_m_s);
        read_value(hg, "lift_vel_m_s", cfg.head_grasp.lift_vel_m_s);
        read_value(hg, "return_vel_m_s", cfg.head_grasp.return_vel_m_s);
        read_value(hg, "pre_grasp_settle_sec", cfg.head_grasp.pre_grasp_settle_sec);
        read_value(hg, "hand_detect_invalid_redo_max", cfg.head_grasp.hand_detect_invalid_redo_max);
        read_hand_offset(hg["hand_grasp"]["left"], cfg.head_grasp.hand_left);
        if (hg["hand_grasp"]["right"])
        {
            read_hand_offset(hg["hand_grasp"]["right"], cfg.head_grasp.hand_right);
        }
        else
        {
            cfg.head_grasp.hand_right.offset_x = cfg.head_grasp.hand_left.offset_x;
            cfg.head_grasp.hand_right.offset_y = -cfg.head_grasp.hand_left.offset_y;
            std::cout << "[cfg] 未读到 hand_grasp.right，用左手 offset_y 取反\n";
        }

        const YAML::Node valid = root["grasp_valid"];
        read_value(valid, "z_min", cfg.grasp_valid.z_min);
        read_value(valid, "z_max", cfg.grasp_valid.z_max);
        read_value(valid, "x_min", cfg.grasp_valid.x_min);
        read_value(valid, "x_max", cfg.grasp_valid.x_max);
        read_value(valid, "right_y_min", cfg.grasp_valid.right_y_min);
        read_value(valid, "right_y_max", cfg.grasp_valid.right_y_max);
        read_value(valid, "left_y_min", cfg.grasp_valid.left_y_min);
        read_value(valid, "left_y_max", cfg.grasp_valid.left_y_max);

        const YAML::Node zone = root["grasp_zone"];
        read_value(zone, "y_side_split", cfg.grasp_zone.y_side_split);
        read_value(zone, "y_max_abs", cfg.grasp_zone.y_max_abs);
        read_value(zone, "x_min", cfg.grasp_zone.x_min);
        read_value(zone, "x_max", cfg.grasp_zone.x_max);
        read_value(zone, "edge_margin_frac", cfg.grasp_zone.edge_margin_frac);
        read_value(zone, "cam_xy_over_z_max", cfg.grasp_zone.cam_xy_over_z_max);
        read_value(zone, "front_row_tolerance_m", cfg.grasp_zone.front_row_tolerance_m);
        read_value(zone, "column_split_y", cfg.grasp_zone.column_split_y);
        read_value(zone, "hole_pitch_m", cfg.grasp_zone.hole_pitch_m);
        read_value(zone, "column_count", cfg.grasp_zone.column_count);
        read_value(zone, "min_simultaneous_col_delta", cfg.grasp_zone.min_simultaneous_col_delta);
        read_value(zone, "far_row_max", cfg.grasp_zone.far_row_max);

        const YAML::Node waist = root["waist"];
        if (!read_pose(waist["layer3_home"], cfg.waist.layer3_home))
            throw std::runtime_error("waist.layer3_home 必须有 6 个数");
        read_value(waist, "stagger_step_x", cfg.waist.stagger_step_x);
        read_value(waist, "stagger_max_steps", cfg.waist.stagger_max_steps);
        read_value(waist, "move_settle_sec", cfg.waist.move_settle_sec);
        read_value(waist, "x_min", cfg.waist.x_min);
        read_value(waist, "x_max", cfg.waist.x_max);
        read_value(waist, "z_min", cfg.waist.z_min);
        read_value(waist, "z_max", cfg.waist.z_max);
        read_value(waist, "grasp_object_z_min", cfg.waist.grasp_object_z_min);
        read_value(waist, "grasp_lower_z", cfg.waist.grasp_lower_z);
        read_value(waist, "ready_z_m", cfg.waist.ready_z_m);
        read_value(waist, "ready_forward_m", cfg.waist.ready_forward_m);
        read_value(waist, "far_row_forward_m", cfg.waist.far_row_forward_m);

        load_conveyor_station(root["conveyor"], cfg.conveyor, "conveyor");
        cfg.conveyor2 = cfg.conveyor;
        load_conveyor_station(root["conveyor2"], cfg.conveyor2, "conveyor2");

        const YAML::Node chassis = root["chassis"];
        read_value(chassis, "host", cfg.chassis.host);
        read_value(chassis, "tray_station", cfg.chassis.tray_station);
        read_value(chassis, "tray2_station", cfg.chassis.tray2_station);
        read_value(chassis, "belt_station", cfg.chassis.belt_station);
        read_value(chassis, "out_station", cfg.chassis.out_station);
        read_value(chassis, "nav_timeout_ms", cfg.chassis.nav_timeout_ms);
        if (cfg.chassis.nav_timeout_ms < 1000)
            cfg.chassis.nav_timeout_ms = 1000;

        read_value(root["stagger"], "head_x_threshold", cfg.stagger.head_x_threshold);
        read_value(root["cameras"], "width", cfg.cameras.width);
        read_value(root["cameras"], "height", cfg.cameras.height);
        read_value(root["cameras"], "head_fps", cfg.cameras.head_fps);
        read_value(root["cameras"], "hand_fps", cfg.cameras.hand_fps);

        int algorithm = cfg.vision_detect.head_grasp;
        read_value(root["vision_detect"], "head_grasp", algorithm);
        cfg.vision_detect.head_grasp = clamp_algorithm_id(algorithm);
        algorithm = cfg.vision_detect.right_hand_grasp;
        read_value(root["vision_detect"], "right_hand_grasp", algorithm);
        cfg.vision_detect.right_hand_grasp = clamp_algorithm_id(algorithm);
        algorithm = cfg.vision_detect.left_hand_grasp;
        read_value(root["vision_detect"], "left_hand_grasp", algorithm);
        cfg.vision_detect.left_hand_grasp = clamp_algorithm_id(algorithm);

        if (cfg.grasp_valid.x_min >= cfg.grasp_valid.x_max ||
            cfg.grasp_valid.z_min >= cfg.grasp_valid.z_max ||
            cfg.grasp_valid.right_y_min >= cfg.grasp_valid.right_y_max ||
            cfg.grasp_valid.left_y_min >= cfg.grasp_valid.left_y_max)
            throw std::runtime_error("grasp_valid 的 min 必须小于 max");
        if (cfg.waist.x_min >= cfg.waist.x_max || cfg.waist.z_min >= cfg.waist.z_max)
            throw std::runtime_error("waist 的 min 必须小于 max");
        if (cfg.waist.stagger_max_steps < 0 || cfg.head_grasp.hand_detect_invalid_redo_max < 0)
            throw std::runtime_error("重试次数/腰进步数不能为负数");
        if (cfg.head_grasp.hand_match_xy_max_m <= 0.0 ||
            cfg.grasp_zone.front_row_tolerance_m < 0.0)
            throw std::runtime_error("目标匹配距离必须为正，前排容差不能为负数");
        if (cfg.head_grasp.hand_xy_refine_max_m < 0.0)
            throw std::runtime_error("hand_xy_refine_max_m 不能为负数");
        if (cfg.tray.z_refine_max_m < 0.0)
            throw std::runtime_error("tray.z_refine_max_m 不能为负数");
        if (cfg.tray.part_above_tray_m < 0.0)
            throw std::runtime_error("tray.part_above_tray_m 不能为负数");
        if (cfg.tray.fuse_frames < 1)
            throw std::runtime_error("tray.fuse_frames 必须 >= 1");
        if (cfg.grasp_zone.hole_pitch_m <= 1e-6 || cfg.grasp_zone.column_count < 2)
            throw std::runtime_error("hole_pitch_m 必须为正，column_count 至少为 2");
        if (cfg.grasp_zone.min_simultaneous_col_delta < 1)
            throw std::runtime_error("min_simultaneous_col_delta 必须 >= 1");
        if (cfg.grasp_zone.far_row_max < 0)
            throw std::runtime_error("far_row_max 不能为负数");
        if (cfg.waist.far_row_forward_m < 0.0)
            throw std::runtime_error("far_row_forward_m 不能为负数");
        if (cfg.waist.ready_forward_m < 0.0)
            throw std::runtime_error("ready_forward_m 不能为负数");
        if (cfg.waist.ready_z_m < cfg.waist.z_min || cfg.waist.ready_z_m > cfg.waist.z_max)
            throw std::runtime_error("ready_z_m 必须在 z_min 与 z_max 之间");
        if (cfg.grasp_valid.right_y_max + 1e-9 < cfg.grasp_zone.column_split_y)
            throw std::runtime_error("grasp_valid.right_y_max 不能小于 column_split_y，否则右三列会被包络裁掉");
        if (cfg.grasp_valid.left_y_min > cfg.grasp_zone.column_split_y + 1e-9)
            throw std::runtime_error("grasp_valid.left_y_min 不能大于 column_split_y，否则左三列会被包络裁掉");
        if (cfg.cameras.width <= 0 || cfg.cameras.height <= 0 ||
            cfg.cameras.head_fps <= 0 || cfg.cameras.hand_fps <= 0)
            throw std::runtime_error("相机分辨率和帧率必须为正数");
        return true;
    }
    catch (const std::exception &ex)
    {
        err = std::string("读取配置失败: ") + ex.what();
        return false;
    }
}

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
    if (row == 1)
        return is_right ? g_move_cfg.tray2_place.right_row6_rpy_deg
                        : g_move_cfg.tray2_place.left_row6_rpy_deg;
    const int g = tray_row_pose_group(row);
    return is_right ? g_move_cfg.tray2_place.right_row_rpy_deg[g]
                    : g_move_cfg.tray2_place.left_row_rpy_deg[g];
}

const GraspRpyDeg &tray2_place_rpy_deg_for_ready(int ready_id, bool is_right)
{
    if (ready_id == 6)
        return is_right ? g_move_cfg.tray2_place.right_row6_rpy_deg
                        : g_move_cfg.tray2_place.left_row6_rpy_deg;
    const int g = std::clamp(ready_id, 1, 3) - 1;
    return is_right ? g_move_cfg.tray2_place.right_row_rpy_deg[g]
                    : g_move_cfg.tray2_place.left_row_rpy_deg[g];
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

void print_move_box_config(const MoveBoxConfig &cfg)
{
    std::cout << "[cfg] standby right=" << cfg.standby.right << "\n"
              << "[cfg] standby left=" << cfg.standby.left << "\n"
              << "[cfg] home_tcp right=" << cfg.home_tcp.right << "\n"
              << "[cfg] home_tcp left=" << cfg.home_tcp.left << "\n"
              << "[cfg] 工作包络 grasp_valid x=[" << cfg.grasp_valid.x_min << ','
              << cfg.grasp_valid.x_max << "] z=[" << cfg.grasp_valid.z_min << ','
              << cfg.grasp_valid.z_max << "] y右=[" << cfg.grasp_valid.right_y_min << ','
              << cfg.grasp_valid.right_y_max << "] y左=[" << cfg.grasp_valid.left_y_min << ','
              << cfg.grasp_valid.left_y_max << "]；ArUco 列已分配时不再用盘心Y拒目标\n"
              << "[cfg] target select=同行最近、满排同时(3-6/2-5/1-4) front_row_tol="
              << cfg.grasp_zone.front_row_tolerance_m
              << "m hand_match_xy_max=" << cfg.head_grasp.hand_match_xy_max_m
              << "m hand_xy_refine_max=" << cfg.head_grasp.hand_xy_refine_max_m << "m\n"
              << "[cfg] columns 分列=ArUco列号(1-3右/4-6左) yaml回退split_y="
              << cfg.grasp_zone.column_split_y
              << (g_have_live_tray_origin_y ? " live盘心y=" : " live盘心y=无 ")
              << (g_have_live_tray_origin_y ? g_live_tray_origin_y : 0.0)
              << " pitch=" << cfg.grasp_zone.hole_pitch_m
              << "m 右列1-" << (cfg.grasp_zone.column_count / 2)
              << " 左列" << (cfg.grasp_zone.column_count / 2 + 1) << "-" << cfg.grasp_zone.column_count
              << " 同时最小列差=" << cfg.grasp_zone.min_simultaneous_col_delta
              << " (如 1-4,2-5,3-6)"
              << " 远三排=row1-" << cfg.grasp_zone.far_row_max
              << " 近三排=row" << (cfg.grasp_zone.far_row_max + 1) << "-6\n"
              << "[cfg] waist home xyz=(" << cfg.waist.layer3_home(0) << ","
              << cfg.waist.layer3_home(1) << "," << cfg.waist.layer3_home(2)
              << ") x=[" << cfg.waist.x_min << "," << cfg.waist.x_max
              << "] z=[" << cfg.waist.z_min << "," << cfg.waist.z_max << "]\n"
              << "[cfg] waist grasp_z_target=" << cfg.waist.grasp_object_z_min
              << "m lower_step=" << cfg.waist.grasp_lower_z << "m z_min=" << cfg.waist.z_min
              << "m ready_z=" << cfg.waist.ready_z_m
              << "m ready_forward=" << cfg.waist.ready_forward_m
              << "m far_row_forward=" << cfg.waist.far_row_forward_m << "m\n";
    print_conveyor_station(cfg.conveyor, "conveyor");
    print_conveyor_station(cfg.conveyor2, "conveyor2");
    std::cout << std::fixed << std::setprecision(4)
              << "[cfg] chassis host=" << cfg.chassis.host
              << " tray=" << cfg.chassis.tray_station
              << " tray2=" << cfg.chassis.tray2_station
              << " belt=" << cfg.chassis.belt_station
              << " out=" << cfg.chassis.out_station
              << " nav_timeout_ms=" << cfg.chassis.nav_timeout_ms << "\n"
              << "[cfg] head yaw=" << cfg.head.yaw_deg
              << " pitch=" << cfg.head.pitch_deg
              << " far_pitch=" << cfg.head.far_pitch_deg
              << " roll=" << cfg.head.roll_deg
              << " deg speed=" << cfg.head.speed_deg_s << " deg/s\n"
              << "[cfg] tray z_ref=" << cfg.tray.z_ref_m
              << "m at waist_z=" << cfg.tray.z_ref_waist_z_m
              << "m part_above=" << cfg.tray.part_above_tray_m
              << "m refine_max=" << cfg.tray.z_refine_max_m
              << "m fuse_frames=" << cfg.tray.fuse_frames << "\n"
              << "[cfg] hand_z hover_above=" << cfg.head_grasp.hover_above_m
              << "m z_offset=" << cfg.head_grasp.grasp_z_offset_m
              << "m nearest_row_z=" << cfg.head_grasp.nearest_row_grasp_z_offset_m
              << "m use_hand_camera=" << (cfg.head_grasp.use_hand_camera ? "on" : "off")
              << " use_bezier=" << (cfg.head_grasp.use_bezier_grasp ? "on" : "off") << "\n"
              << "[cfg] grasp offset left xyz=(" << cfg.head_grasp.goal_x_offset
              << "," << cfg.head_grasp.goal_y_offset << ","
              << cfg.head_grasp.grasp_z_offset_m << ") right xyz=("
              << cfg.head_grasp.right_goal_x_offset << ","
              << cfg.head_grasp.right_goal_y_offset << ","
              << cfg.head_grasp.right_grasp_z_offset_m << ")"
              << "m hand_xy left=(" << cfg.head_grasp.hand_left.offset_x << ","
              << cfg.head_grasp.hand_left.offset_y << ") right=("
              << cfg.head_grasp.hand_right.offset_x << ","
              << cfg.head_grasp.hand_right.offset_y << ")\n"
              << "[cfg] row_rpy left"
              << " ready1(row3/6)=(" << cfg.head_grasp.left_row_rpy_deg[0].rx << ","
              << cfg.head_grasp.left_row_rpy_deg[0].ry << ","
              << cfg.head_grasp.left_row_rpy_deg[0].rz << ")"
              << " ready2(row2/5)=(" << cfg.head_grasp.left_row_rpy_deg[1].rx << ","
              << cfg.head_grasp.left_row_rpy_deg[1].ry << ","
              << cfg.head_grasp.left_row_rpy_deg[1].rz << ")"
              << " ready3(row4)=(" << cfg.head_grasp.left_row_rpy_deg[2].rx << ","
              << cfg.head_grasp.left_row_rpy_deg[2].ry << ","
              << cfg.head_grasp.left_row_rpy_deg[2].rz << ")"
              << " row1最远=(" << cfg.head_grasp.left_row6_rpy_deg.rx << ","
              << cfg.head_grasp.left_row6_rpy_deg.ry << ","
              << cfg.head_grasp.left_row6_rpy_deg.rz << ") deg\n"
              << "[cfg] row_rpy right"
              << " ready1(row3/6)=(" << cfg.head_grasp.right_row_rpy_deg[0].rx << ","
              << cfg.head_grasp.right_row_rpy_deg[0].ry << ","
              << cfg.head_grasp.right_row_rpy_deg[0].rz << ")"
              << " ready2(row2/5)=(" << cfg.head_grasp.right_row_rpy_deg[1].rx << ","
              << cfg.head_grasp.right_row_rpy_deg[1].ry << ","
              << cfg.head_grasp.right_row_rpy_deg[1].rz << ")"
              << " ready3(row4)=(" << cfg.head_grasp.right_row_rpy_deg[2].rx << ","
              << cfg.head_grasp.right_row_rpy_deg[2].ry << ","
              << cfg.head_grasp.right_row_rpy_deg[2].rz << ")"
              << " row1最远=(" << cfg.head_grasp.right_row6_rpy_deg.rx << ","
              << cfg.head_grasp.right_row6_rpy_deg.ry << ","
              << cfg.head_grasp.right_row6_rpy_deg.rz << ") deg\n"
              << "[cfg] tray2_place left"
              << " ready1=(" << cfg.tray2_place.left_row_rpy_deg[0].rx << ","
              << cfg.tray2_place.left_row_rpy_deg[0].ry << ","
              << cfg.tray2_place.left_row_rpy_deg[0].rz << ")"
              << " ready2=(" << cfg.tray2_place.left_row_rpy_deg[1].rx << ","
              << cfg.tray2_place.left_row_rpy_deg[1].ry << ","
              << cfg.tray2_place.left_row_rpy_deg[1].rz << ")"
              << " ready3=(" << cfg.tray2_place.left_row_rpy_deg[2].rx << ","
              << cfg.tray2_place.left_row_rpy_deg[2].ry << ","
              << cfg.tray2_place.left_row_rpy_deg[2].rz << ")"
              << " row1=(" << cfg.tray2_place.left_row6_rpy_deg.rx << ","
              << cfg.tray2_place.left_row6_rpy_deg.ry << ","
              << cfg.tray2_place.left_row6_rpy_deg.rz << ") deg\n"
              << "[cfg] tray2_place right"
              << " ready1=(" << cfg.tray2_place.right_row_rpy_deg[0].rx << ","
              << cfg.tray2_place.right_row_rpy_deg[0].ry << ","
              << cfg.tray2_place.right_row_rpy_deg[0].rz << ")"
              << " ready2=(" << cfg.tray2_place.right_row_rpy_deg[1].rx << ","
              << cfg.tray2_place.right_row_rpy_deg[1].ry << ","
              << cfg.tray2_place.right_row_rpy_deg[1].rz << ")"
              << " ready3=(" << cfg.tray2_place.right_row_rpy_deg[2].rx << ","
              << cfg.tray2_place.right_row_rpy_deg[2].ry << ","
              << cfg.tray2_place.right_row_rpy_deg[2].rz << ")"
              << " row1=(" << cfg.tray2_place.right_row6_rpy_deg.rx << ","
              << cfg.tray2_place.right_row6_rpy_deg.ry << ","
              << cfg.tray2_place.right_row6_rpy_deg.rz << ") deg\n"
              << "[cfg] tray2 offset left xyz=(" << cfg.tray2_place.goal_x_offset
              << "," << cfg.tray2_place.goal_y_offset << ","
              << cfg.tray2_place.grasp_z_offset_m << ") right xyz=("
              << cfg.tray2_place.right_goal_x_offset << ","
              << cfg.tray2_place.right_goal_y_offset << ","
              << cfg.tray2_place.right_grasp_z_offset_m
              << ") nearest_row_z=" << cfg.tray2_place.nearest_row_z_offset_m << "\n"
              << "[cfg] bezier B_above_C=" << cfg.head_grasp.bezier_guide_height_m
              << "m A_below_B=" << cfg.head_grasp.bezier_ab_gap_m
              << "m vel_max=" << cfg.head_grasp.bezier_vel_m_s
              << "m/s orient_hold=" << cfg.head_grasp.bezier_orient_finish_ratio
              << " endpoint_tol=" << cfg.head_grasp.bezier_endpoint_xyz_tol_m << "m/"
              << cfg.head_grasp.bezier_endpoint_rpy_tol_deg << "deg\n"
              << "[cfg] arm vel approach=" << cfg.head_grasp.approach_vel_m_s
              << " descend=" << cfg.head_grasp.descend_vel_m_s
              << " lift=" << cfg.head_grasp.lift_vel_m_s
              << " return=" << cfg.head_grasp.return_vel_m_s
              << " m/s pre_grasp_settle=" << cfg.head_grasp.pre_grasp_settle_sec << "s\n"
              << "[cfg] cameras " << cfg.cameras.width << 'x' << cfg.cameras.height
              << " head=" << cfg.cameras.head_fps << "fps hand=" << cfg.cameras.hand_fps << "fps\n";
}
