#include "hand_shadow.h"

#include "Ti5_socketcan.h"
#include "function.h"
#include "move_box_config.h"
#include "seg_pose_bridge.h"

#include <opencv2/imgproc.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

namespace
{

struct QSample
{
    std::atomic<std::uint32_t> seq{0};
    double q[7]{};
    std::int64_t t_ns = 0;
    bool valid = false;
};

struct Slot
{
    std::mutex mu;
    bool active = false;
    std::uint32_t gen = 0;
    Robot_Arm *arm = nullptr;
    Eigen::Matrix<double, 1, 6> goal = Eigen::Matrix<double, 1, 6>::Zero();
    std::string tag;
};

struct Eye
{
    bool ok = false;
    std::array<double, 16> T{};
    std::string err;
};

QSample g_q[2];
Slot g_slot[2];
Eye g_eye[2];
bool g_eye_loaded = false;
std::mutex g_eye_mu;

std::atomic<SegPoseBridge *> g_bridge{nullptr};
std::atomic<bool> g_run{false};
std::mutex g_wait_mu;
std::condition_variable g_cv;
std::thread g_thread;
std::mutex g_log_mu;
int g_turn = 0;

std::int64_t steady_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

const char *hand_name(int hand)
{
    return hand == 1 ? "右手" : "左手";
}

CameraSlot hand_slot(int hand)
{
    return hand == 1 ? CameraSlot::RightHand : CameraSlot::LeftHand;
}

int image_rotation_deg(int hand)
{
    // 2026-10-21 标定：左手图像转 180°，右手不转。
    return hand == 0 ? 180 : 0;
}

void rotate_frame_180(CameraFrameData &frame)
{
    const int w = frame.width;
    const int h = frame.height;
    if (w <= 0 || h <= 0 || frame.color_bgr.empty())
        return;

    cv::Mat bgr(h, w, CV_8UC3, frame.color_bgr.data());
    cv::Mat bgr_rot;
    cv::rotate(bgr, bgr_rot, cv::ROTATE_180);
    frame.color_bgr.resize(static_cast<std::size_t>(h) * static_cast<std::size_t>(w) * 3u);
    if (bgr_rot.isContinuous())
        std::memcpy(frame.color_bgr.data(), bgr_rot.ptr(), frame.color_bgr.size());

    if (frame.depth_m.size() == static_cast<std::size_t>(h) * static_cast<std::size_t>(w))
    {
        cv::Mat depth(h, w, CV_32FC1, frame.depth_m.data());
        cv::Mat depth_rot;
        cv::rotate(depth, depth_rot, cv::ROTATE_180);
        if (depth_rot.isContinuous())
            std::memcpy(frame.depth_m.data(), depth_rot.ptr<float>(),
                        frame.depth_m.size() * sizeof(float));
    }

    frame.K[2] = static_cast<double>(w - 1) - frame.K[2];
    frame.K[5] = static_cast<double>(h - 1) - frame.K[5];
    frame.dist[2] = -frame.dist[2];
    frame.dist[3] = -frame.dist[3];
}

bool read_q(int hand, double q[7], double &age_ms)
{
    if (hand < 0 || hand > 1)
        return false;
    QSample &sample = g_q[hand];
    for (int attempt = 0; attempt < 8; ++attempt)
    {
        const std::uint32_t a = sample.seq.load(std::memory_order_acquire);
        if ((a & 1u) != 0u)
            continue;
        double tmp[7];
        for (int i = 0; i < 7; ++i)
            tmp[i] = sample.q[i];
        const std::int64_t t_ns = sample.t_ns;
        const bool valid = sample.valid;
        const std::uint32_t b = sample.seq.load(std::memory_order_acquire);
        if (a != b || (b & 1u) != 0u)
            continue;
        if (!valid)
            return false;
        for (int i = 0; i < 7; ++i)
            q[i] = tmp[i];
        const double dt_ms = static_cast<double>(steady_ns() - t_ns) / 1.0e6;
        age_ms = dt_ms < 0.0 ? 0.0 : dt_ms;
        return true;
    }
    return false;
}

void ensure_eye()
{
    std::lock_guard<std::mutex> lock(g_eye_mu);
    if (g_eye_loaded)
        return;
    g_eye_loaded = true;
    const std::string paths[2] = {
        default_left_hand_to_robot_yaml_path(),
        default_right_hand_to_robot_yaml_path(),
    };
    for (int hand = 0; hand < 2; ++hand)
    {
        if (!load_cam2robot_matrix(paths[hand], g_eye[hand].T, g_eye[hand].err))
            g_eye[hand].ok = false;
        else
            g_eye[hand].ok = true;
    }
}

Eigen::Matrix4d row_major_matrix(const std::array<double, 16> &pose)
{
    Eigen::Matrix4d T;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            T(r, c) = pose[static_cast<std::size_t>(r * 4 + c)];
    return T;
}

std::string csv_cell(std::string text)
{
    for (char &ch : text)
    {
        if (ch == ',' || ch == '\n' || ch == '\r')
            ch = ' ';
    }
    return text;
}

std::string now_text()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t sec = std::chrono::system_clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch()) %
                    1000;
    std::tm local{};
    localtime_r(&sec, &local);
    std::ostringstream out;
    out << std::put_time(&local, "%Y-%m-%d %H:%M:%S") << '.'
        << std::setw(3) << std::setfill('0') << ms.count();
    return out.str();
}

void append_log(const std::string &line)
{
    std::lock_guard<std::mutex> lock(g_log_mu);
    const std::filesystem::path path =
        std::filesystem::path(project_root_dir()) / "logs" / "hand_shadow.csv";
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
    const bool exists = std::filesystem::exists(path, ec);
    const bool need_header = !exists || std::filesystem::file_size(path, ec) == 0;
    std::ofstream out(path, std::ios::app);
    if (!out)
        return;
    if (need_header)
    {
        out << "时间,手,阶段,目标x_m,目标y_m,目标z_m,目标rx,目标ry,目标rz,"
               "指令x_m,指令y_m,指令z_m,指令dx_mm,指令dy_mm,指令dz_mm,关节滞后_ms,"
               "q0,q1,q2,q3,q4,q5,q6,取帧_ms,推理_ms,图像旋转_deg,宽,高,"
               "检测数,类别,类名,置信度,成功,框u,框v,相机x_m,相机y_m,相机z_m,"
               "视觉x_m,视觉y_m,视觉z_m,视觉dx_mm,视觉dy_mm,视觉dz_mm,视觉dxy_mm,"
               "判定,原因,候选,算法信息\n";
    }
    out << line << '\n';
}

struct Solved
{
    bool have = false;
    int class_id = -1;
    std::string class_name;
    float confidence = 0.f;
    bool success = false;
    float box_u = -1.f;
    float box_v = -1.f;
    double cam_x = 0;
    double cam_y = 0;
    double cam_z = 0;
    double base_x = 0;
    double base_y = 0;
    double base_z = 0;
    double dxy_m = 0;
    std::string message;
};

void observe_hand(int hand)
{
    Slot &slot = g_slot[hand];
    std::uint32_t gen = 0;
    Eigen::Matrix<double, 1, 6> goal = Eigen::Matrix<double, 1, 6>::Zero();
    std::string tag;
    {
        std::lock_guard<std::mutex> lock(slot.mu);
        if (!slot.active || slot.arm == nullptr)
            return;
        gen = slot.gen;
        goal = slot.goal;
        tag = slot.tag;
    }

    SegPoseBridge *bridge = g_bridge.load(std::memory_order_acquire);
    if (bridge == nullptr || !g_run.load(std::memory_order_acquire))
        return;

    const auto grab_t0 = std::chrono::steady_clock::now();
    CameraFrameData frame = RealSenseMultiCam::live() == nullptr
                                ? CameraFrameData{}
                                : RealSenseMultiCam::live()->grab(hand_slot(hand));
    const double grab_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - grab_t0)
                               .count();
    if (!frame.ok && frame.message.empty())
        frame.message = "手相机未启动";

    double q[7] = {};
    double q_age_ms = -1.0;
    const bool have_q = read_q(hand, q, q_age_ms);
    const int rot = image_rotation_deg(hand);
    if (frame.ok && rot == 180)
        rotate_frame_180(frame);

    PoseRunResult result;
    double infer_ms = -1.0;
    if (frame.ok && g_run.load(std::memory_order_acquire))
    {
        const auto infer_t0 = std::chrono::steady_clock::now();
        try
        {
            result = bridge->run_yolo(
                frame, algorithm_id_for_slot(hand_slot(hand)), false, hand_slot(hand));
        }
        catch (const std::exception &ex)
        {
            result.ok = false;
            result.message = ex.what();
        }
        infer_ms = std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - infer_t0)
                       .count();
    }
    else if (!frame.ok)
    {
        result.message = frame.message;
    }

    ensure_eye();
    Eye eye;
    {
        std::lock_guard<std::mutex> lock(g_eye_mu);
        eye = g_eye[hand];
    }

    const double match_m = g_move_cfg.head_grasp.hand_match_xy_max_m;
    Solved best;
    std::ostringstream candidates;
    int n_det = 0;
    bool have_tcp = false;
    Eigen::Matrix<double, 1, 6> tcp = Eigen::Matrix<double, 1, 6>::Zero();
    std::string skip;

    {
        std::lock_guard<std::mutex> lock(slot.mu);
        if (!slot.active || slot.gen != gen || slot.arm == nullptr)
            skip = "伸手已结束";
        else if (!have_q)
            skip = "还没有指令关节";
        else
        {
            try
            {
                Eigen::RowVectorXd qv(7);
                for (int i = 0; i < 7; ++i)
                    qv(i) = q[i];
                tcp = arm_tcp_from_joints(*slot.arm, qv);
                have_tcp = tcp.allFinite();
                if (result.ok && eye.ok)
                {
                    for (const PoseTargetResult &target : result.targets)
                    {
                        ++n_det;
                        if (!target.success)
                        {
                            candidates << target.class_id << ':' << target.confidence << ":失败;";
                            continue;
                        }
                        const std::array<double, 16> flange =
                            transform_pose_cam_to_robot(eye.T, target.pose_4x4);
                        Eigen::Matrix<double, 1, 6> flange_pose =
                            T2PosEulerAngles(row_major_matrix(flange));
                        const Eigen::Matrix<double, 1, 6> base =
                            arm_base_visual_from_joints(*slot.arm, qv, flange_pose);
                        if (!base.head<3>().allFinite())
                        {
                            candidates << target.class_id << ':' << target.confidence << ":无效;";
                            continue;
                        }
                        const double dxy = std::hypot(base(0) - goal(0), base(1) - goal(1));
                        candidates << target.class_id << ':' << std::fixed << std::setprecision(2)
                                   << target.confidence << ':' << std::setprecision(1)
                                   << (dxy * 1000.0) << "mm;";
                        if (!best.have || dxy < best.dxy_m)
                        {
                            best.have = true;
                            best.class_id = target.class_id;
                            best.class_name = target.class_name;
                            best.confidence = target.confidence;
                            best.success = target.success;
                            best.box_u = target.box_u;
                            best.box_v = target.box_v;
                            best.cam_x = target.pose_4x4[3];
                            best.cam_y = target.pose_4x4[7];
                            best.cam_z = target.pose_4x4[11];
                            best.base_x = base(0);
                            best.base_y = base(1);
                            best.base_z = base(2);
                            best.dxy_m = dxy;
                            best.message = target.message;
                        }
                    }
                }
            }
            catch (const std::exception &ex)
            {
                skip = ex.what();
            }
        }
    }

    if (skip == "伸手已结束")
        return;
    if (!result.ok && skip.empty())
        skip = result.message.empty() ? "推理失败" : result.message;
    else if (!eye.ok && skip.empty())
        skip = eye.err.empty() ? "手眼读取失败" : eye.err;
    else if (result.ok && n_det == 0 && skip.empty())
        skip = "未检测到目标";
    else if (result.ok && !best.have && skip.empty())
        skip = "没有成功的位姿";

    const bool accept = best.have && skip.empty() && best.dxy_m <= match_m;
    std::string verdict = "无观测";
    std::string reason = skip;
    if (best.have && skip.empty())
    {
        if (accept)
        {
            verdict = "接受";
        }
        else
        {
            verdict = "拒绝";
            std::ostringstream why;
            why << std::fixed << std::setprecision(1)
                << "偏离执行目标 " << (best.dxy_m * 1000.0) << " mm，门限 "
                << (match_m * 1000.0) << " mm";
            reason = why.str();
        }
    }
    else if (!reason.empty())
    {
        verdict = "拒绝";
    }

    const double dx_mm = best.have ? (best.base_x - goal(0)) * 1000.0 : 0.0;
    const double dy_mm = best.have ? (best.base_y - goal(1)) * 1000.0 : 0.0;
    const double dz_mm = best.have ? (best.base_z - goal(2)) * 1000.0 : 0.0;
    const double cmd_dx = have_tcp ? (tcp(0) - goal(0)) * 1000.0 : 0.0;
    const double cmd_dy = have_tcp ? (tcp(1) - goal(1)) * 1000.0 : 0.0;
    const double cmd_dz = have_tcp ? (tcp(2) - goal(2)) * 1000.0 : 0.0;

    std::cout << std::fixed << std::setprecision(1);
    if (best.have)
    {
        std::cout << "[影子] " << hand_name(hand) << ' ' << tag
                  << "  视觉-目标 dx=" << dx_mm << " dy=" << dy_mm << " dz=" << dz_mm
                  << " mm  dxy=" << (best.dxy_m * 1000.0) << " mm  " << verdict;
        if (!reason.empty())
            std::cout << "  " << reason;
        std::cout << "  置信度 " << std::setprecision(2) << best.confidence
                  << "  class=" << best.class_id;
        if (n_det > 1)
            std::cout << "  候选" << n_det;
        std::cout << '\n';
    }
    else
    {
        std::cout << "[影子] " << hand_name(hand) << ' ' << tag
                  << "  无有效目标";
        if (!reason.empty())
            std::cout << "  " << reason;
        std::cout << "  检测数 " << n_det << '\n';
    }
    std::cout << "[影子] " << hand_name(hand) << ' ' << tag
              << "  推理 " << std::setprecision(0) << infer_ms
              << " ms  取帧 " << grab_ms << " ms";
    if (have_q)
        std::cout << "  关节滞后 " << std::setprecision(0) << q_age_ms << " ms";
    if (have_tcp)
    {
        std::cout << std::setprecision(1)
                  << "  指令TCP-目标 dx=" << cmd_dx << " dy=" << cmd_dy
                  << " dz=" << cmd_dz << " mm";
    }
    std::cout << std::endl;

    std::ostringstream row;
    row << std::fixed << std::setprecision(4)
        << now_text() << ',' << hand_name(hand) << ',' << csv_cell(tag) << ','
        << goal(0) << ',' << goal(1) << ',' << goal(2) << ','
        << goal(3) << ',' << goal(4) << ',' << goal(5) << ',';
    if (have_tcp)
        row << tcp(0) << ',' << tcp(1) << ',' << tcp(2) << ','
            << cmd_dx << ',' << cmd_dy << ',' << cmd_dz << ',';
    else
        row << ",,,,,,";
    row << std::setprecision(1) << q_age_ms << ',';
    row << std::setprecision(5);
    for (int i = 0; i < 7; ++i)
        row << q[i] << ',';
    row << std::setprecision(1) << grab_ms << ',' << infer_ms << ',' << rot << ','
        << frame.width << ',' << frame.height << ',' << n_det << ','
        << best.class_id << ',' << csv_cell(best.class_name) << ','
        << std::setprecision(3) << best.confidence << ','
        << (best.success ? 1 : 0) << ','
        << std::setprecision(1) << best.box_u << ',' << best.box_v << ','
        << std::setprecision(4) << best.cam_x << ',' << best.cam_y << ',' << best.cam_z << ',';
    if (best.have)
        row << best.base_x << ',' << best.base_y << ',' << best.base_z << ','
            << std::setprecision(2) << dx_mm << ',' << dy_mm << ',' << dz_mm << ','
            << (best.dxy_m * 1000.0) << ',';
    else
        row << ",,,,,,,";
    row << verdict << ',' << csv_cell(reason) << ',' << csv_cell(candidates.str()) << ','
        << csv_cell(result.message);
    append_log(row.str());
}

int pick_hand()
{
    for (int step = 0; step < 2; ++step)
    {
        const int hand = (g_turn + step) % 2;
        std::lock_guard<std::mutex> lock(g_slot[hand].mu);
        if (g_slot[hand].active)
        {
            g_turn = hand + 1;
            return hand;
        }
    }
    return -1;
}

void shadow_loop()
{
    while (g_run.load(std::memory_order_acquire))
    {
        if (hardware_abort_requested())
        {
            std::unique_lock<std::mutex> lock(g_wait_mu);
            g_cv.wait_for(lock, std::chrono::milliseconds(100), [] {
                return !g_run.load(std::memory_order_acquire);
            });
            continue;
        }
        const int hand = pick_hand();
        if (hand < 0)
        {
            std::unique_lock<std::mutex> lock(g_wait_mu);
            g_cv.wait_for(lock, std::chrono::milliseconds(100), [] {
                return !g_run.load(std::memory_order_acquire);
            });
            continue;
        }
        try
        {
            observe_hand(hand);
        }
        catch (const std::exception &ex)
        {
            std::cerr << "[影子] 观测失败: " << ex.what() << '\n';
        }
    }
}

} // namespace

void hand_shadow_bind(SegPoseBridge *bridge)
{
    if (bridge == nullptr)
    {
        g_run.store(false, std::memory_order_release);
        g_cv.notify_all();
        if (g_thread.joinable())
            g_thread.join();
        g_bridge.store(nullptr, std::memory_order_release);
        return;
    }
    if (!g_move_cfg.cameras.hand_shadow)
    {
        std::cout << "[影子] 已关（cameras.hand_shadow=false），伸手不取帧、不推理\n"
                  << std::flush;
        return;
    }
    g_bridge.store(bridge, std::memory_order_release);
    if (!g_thread.joinable())
    {
        g_run.store(true, std::memory_order_release);
        g_thread = std::thread(shadow_loop);
    }
    std::cout << "[影子] 手相机影子模式已开。伸手时取帧、推理、解算并写 logs/hand_shadow.csv，"
                 "不下发电机位置。左手图像按标定转 180°，右手不转。\n"
              << std::flush;
}

void hand_shadow_note_q(int hand, const double q_rad[7])
{
    if (!g_run.load(std::memory_order_acquire))
        return;
    if (hand < 0 || hand > 1 || q_rad == nullptr)
        return;
    QSample &sample = g_q[hand];
    const std::uint32_t seq = sample.seq.load(std::memory_order_relaxed);
    sample.seq.store(seq + 1u, std::memory_order_release);
    for (int i = 0; i < 7; ++i)
        sample.q[i] = q_rad[i];
    sample.t_ns = steady_ns();
    sample.valid = true;
    sample.seq.store(seq + 2u, std::memory_order_release);
}

void hand_shadow_begin(
    int hand,
    Robot_Arm *arm,
    const Eigen::Matrix<double, 1, 6> &goal,
    const char *tag)
{
    if (!g_run.load(std::memory_order_acquire))
        return;
    if (hand < 0 || hand > 1 || arm == nullptr)
        return;
    if (g_bridge.load(std::memory_order_acquire) == nullptr)
    {
        std::cout << "[影子] 算法引擎还没绑定，这次伸手不观测\n" << std::flush;
        return;
    }
    {
        std::lock_guard<std::mutex> lock(g_slot[hand].mu);
        g_slot[hand].arm = arm;
        g_slot[hand].goal = goal;
        g_slot[hand].tag = tag != nullptr ? tag : "";
        g_slot[hand].gen += 1u;
        g_slot[hand].active = true;
    }
    std::cout << std::fixed << std::setprecision(4)
              << "[影子] " << hand_name(hand) << ' ' << (tag != nullptr ? tag : "")
              << " 开始  目标 (" << goal(0) << ", " << goal(1) << ", " << goal(2)
              << ") m  只记录，不下发\n"
              << std::flush;
    g_cv.notify_all();
}

void hand_shadow_end(int hand)
{
    if (!g_run.load(std::memory_order_acquire))
        return;
    if (hand < 0 || hand > 1)
        return;
    std::string tag;
    bool was = false;
    {
        std::lock_guard<std::mutex> lock(g_slot[hand].mu);
        was = g_slot[hand].active;
        tag = g_slot[hand].tag;
        g_slot[hand].active = false;
        g_slot[hand].arm = nullptr;
        g_slot[hand].gen += 1u;
    }
    if (!was)
        return;
    std::cout << "[影子] " << hand_name(hand) << ' ' << tag << " 结束\n" << std::flush;
}
