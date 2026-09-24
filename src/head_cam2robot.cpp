#include "head_cam2robot.h"

#include "head_control.h"
#include "move_box_config.h"
#include "seg_pose_bridge.h"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <mutex>

namespace
{

constexpr double kDeg = M_PI / 180.0;

std::mutex g_calib_mu;
bool g_calib_ok = false;
double g_link_length_mm = 162.0;
double g_T_head_cam[16] = {};

void mul4(const double a[16], const double b[16], double c[16])
{
    for (int r = 0; r < 4; ++r)
    {
        for (int col = 0; col < 4; ++col)
        {
            double s = 0.0;
            for (int k = 0; k < 4; ++k)
                s += a[r * 4 + k] * b[k * 4 + col];
            c[r * 4 + col] = s;
        }
    }
}

void euler_zyx_to_R(double roll_deg, double pitch_deg, double yaw_deg, double R[9])
{
    const double rx = roll_deg * kDeg;
    const double ry = pitch_deg * kDeg;
    const double rz = yaw_deg * kDeg;
    const double cx = std::cos(rx), sx = std::sin(rx);
    const double cy = std::cos(ry), sy = std::sin(ry);
    const double cz = std::cos(rz), sz = std::sin(rz);
    // R = Rz @ Ry @ Rx，与 free_calib / ti5 标定软件一致
    const double rz_m[9] = {cz, -sz, 0.0, sz, cz, 0.0, 0.0, 0.0, 1.0};
    const double ry_m[9] = {cy, 0.0, sy, 0.0, 1.0, 0.0, -sy, 0.0, cy};
    const double rx_m[9] = {1.0, 0.0, 0.0, 0.0, cx, -sx, 0.0, sx, cx};
    double tmp[9] = {};
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
        {
            tmp[r * 3 + c] = 0.0;
            for (int k = 0; k < 3; ++k)
                tmp[r * 3 + c] += rz_m[r * 3 + k] * ry_m[k * 3 + c];
        }
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
        {
            R[r * 3 + c] = 0.0;
            for (int k = 0; k < 3; ++k)
                R[r * 3 + c] += tmp[r * 3 + k] * rx_m[k * 3 + c];
        }
}

bool load_free_calib(std::string &err)
{
    std::lock_guard<std::mutex> lock(g_calib_mu);
    if (g_calib_ok)
        return true;

    const std::string params_path = project_root_dir() + "/free_calib/config/head_params.yaml";
    YAML::Node params;
    try
    {
        params = YAML::LoadFile(params_path);
    }
    catch (const std::exception &e)
    {
        err = std::string("读 free_calib head_params 失败: ") + e.what();
        return false;
    }

    g_link_length_mm = params["link_length_mm"] ? params["link_length_mm"].as<double>() : 162.0;
    std::string calib_name = "camera_to_head_connector_result.yaml";
    if (params["camera_to_head_yaml"])
        calib_name = params["camera_to_head_yaml"].as<std::string>();
    const std::string calib_path = project_root_dir() + "/free_calib/config/" + calib_name;

    YAML::Node calib;
    try
    {
        calib = YAML::LoadFile(calib_path);
    }
    catch (const std::exception &e)
    {
        err = std::string("读 T_head_cam 失败: ") + e.what();
        return false;
    }

    YAML::Node mat = calib["a. matrix"] ? calib["a. matrix"] : calib["matrix"];
    if (!mat || !mat.IsSequence() || mat.size() != 4)
    {
        err = "T_head_cam 缺少 4x4 matrix";
        return false;
    }
    for (int r = 0; r < 4; ++r)
    {
        if (!mat[r] || !mat[r].IsSequence() || mat[r].size() != 4)
        {
            err = "T_head_cam 行不是 4 个数";
            return false;
        }
        for (int c = 0; c < 4; ++c)
            g_T_head_cam[r * 4 + c] = mat[r][c].as<double>();
    }

    g_calib_ok = true;
    std::cout << std::fixed << std::setprecision(1)
              << "[head_ext] 已加载 free_calib T_head_cam  link=" << g_link_length_mm << " mm\n";
    return true;
}

} // namespace

bool compute_head_cam2robot(
    double roll_deg,
    double pitch_deg,
    double yaw_deg,
    std::array<double, 16> &out_m,
    std::string &err)
{
    err.clear();
    if (!load_free_calib(err))
        return false;

    double R[9] = {};
    euler_zyx_to_R(roll_deg, pitch_deg, yaw_deg, R);
    double T_base_head[16] = {
        R[0], R[1], R[2], 0.0,
        R[3], R[4], R[5], 0.0,
        R[6], R[7], R[8], g_link_length_mm,
        0.0, 0.0, 0.0, 1.0};
    double T_base_cam[16] = {};
    {
        std::lock_guard<std::mutex> lock(g_calib_mu);
        mul4(T_base_head, g_T_head_cam, T_base_cam);
    }
    for (int i = 0; i < 16; ++i)
        out_m[static_cast<size_t>(i)] = T_base_cam[i];
    out_m[3] /= 1000.0;
    out_m[7] /= 1000.0;
    out_m[11] /= 1000.0;
    return true;
}

bool load_head_cam2robot(std::array<double, 16> &out_m, std::string &err)
{
    double yaw = g_move_cfg.head.yaw_deg;
    double pitch = g_move_cfg.head.pitch_deg;
    double roll = g_move_cfg.head.roll_deg;
    const bool from_enc = read_head_rpy_deg(yaw, pitch, roll);
    if (compute_head_cam2robot(roll, pitch, yaw, out_m, err))
    {
        std::cout << std::fixed << std::setprecision(2)
                  << "[head_ext] free_calib " << (from_enc ? "编码器" : "yaml头角")
                  << " rpy_deg=(" << yaw << "," << pitch << "," << roll << ")"
                  << std::setprecision(4)
                  << " t_m=(" << out_m[3] << "," << out_m[7] << "," << out_m[11] << ")\n";
        return true;
    }
    const std::string compute_err = err;
    if (load_cam2robot_matrix(default_camera_to_robot_yaml_path(), out_m, err))
    {
        std::cerr << "[head_ext] free_calib 失败(" << compute_err
                  << ")，回退静态 camera_to_base_result.yaml\n";
        err.clear();
        return true;
    }
    err = compute_err + "；静态 yaml: " + err;
    return false;
}
