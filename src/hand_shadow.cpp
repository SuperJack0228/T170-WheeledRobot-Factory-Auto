#include "hand_shadow.h"

#include "Ti5_socketcan.h"
#include "function.h"
#include "move_box_config.h"
#include "seg_pose_bridge.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

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
    int want_class = -1;
    bool hole_locked = false;
    double lock_x = 0.0;
    double lock_y = 0.0;
    float lock_u = -1.f;
    float lock_v = -1.f;
    int lock_index = -1;
    int xy_shots = 0;
    double xy_shot_progress = -1.0;
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
std::atomic<bool> g_watch{false};
std::mutex g_wait_mu;
std::condition_variable g_cv;
std::thread g_threads[2];
std::mutex g_log_mu;
std::mutex g_infer_mu;
std::atomic<bool> g_raw_descent{false};
std::atomic<int> g_raw_seq{0};
std::mutex g_start_mu;

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

void append_csv(const std::filesystem::path &path, const char *header, const std::string &line)
{
    std::lock_guard<std::mutex> lock(g_log_mu);
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
    const bool exists = std::filesystem::exists(path, ec);
    const bool need_header = !exists || std::filesystem::file_size(path, ec) == 0;
    std::ofstream out(path, std::ios::app);
    if (!out)
        return;
    if (need_header && header != nullptr)
        out << header;
    out << line << '\n';
}

void append_log(const std::string &line)
{
    const std::filesystem::path path =
        std::filesystem::path(project_root_dir()) / "logs" / "hand_shadow.csv";
    append_csv(
        path,
        "时间,手,阶段,目标x_m,目标y_m,目标z_m,目标rx,目标ry,目标rz,"
        "指令x_m,指令y_m,指令z_m,指令dx_mm,指令dy_mm,指令dz_mm,关节滞后_ms,"
        "q0,q1,q2,q3,q4,q5,q6,取帧_ms,推理_ms,图像旋转_deg,宽,高,"
        "检测数,类别,类名,置信度,成功,框u,框v,相机x_m,相机y_m,相机z_m,"
        "视觉x_m,视觉y_m,视觉z_m,视觉dx_mm,视觉dy_mm,视觉dz_mm,视觉dxy_mm,"
        "判定,原因,候选,算法信息\n",
        line);
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

struct HoleHit
{
    int index = -1;
    int class_id = -1;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double d_tcp = 0.0;
    double d_goal = 0.0;
    float confidence = 0.f;
    float box_u = -1.f;
    float box_v = -1.f;
    float box_x0 = -1.f;
    float box_y0 = -1.f;
    float box_x1 = -1.f;
    float box_y1 = -1.f;
};

const char *target_noun(int want_class)
{
    if (want_class == 1)
        return "孔";
    if (want_class == 0)
        return "工件";
    return "目标";
}

void log_hand_eye_bias(
    int hand,
    const std::string &tag,
    const Eigen::Matrix<double, 1, 6> &goal,
    const Eigen::Matrix<double, 1, 6> &tcp,
    const std::vector<HoleHit> &hits,
    int want_class)
{
    const bool rewrite = g_move_cfg.cameras.hand_descent_correct;
    const int n = static_cast<int>(hits.size());
    const char *noun = target_noun(want_class);
    const std::filesystem::path path =
        std::filesystem::path(project_root_dir()) / "logs" / "hand_eye_bias.csv";
    const char *header =
        "时间,手,阶段,下发修正,孔数,指尖x_m,指尖y_m,指尖z_m,"
        "目标x_m,目标y_m,目标z_m,检测序号,类别,"
        "基座x_m,基座y_m,基座z_m,误差x_m,误差y_m,误差z_m,误差xy_m\n";

    auto write_row = [&](const HoleHit *hit) {
        std::ostringstream row;
        row << std::fixed << std::setprecision(4)
            << now_text() << ',' << hand_name(hand) << ',' << csv_cell(tag) << ','
            << (rewrite ? 1 : 0) << ',' << n << ','
            << tcp(0) << ',' << tcp(1) << ',' << tcp(2) << ','
            << goal(0) << ',' << goal(1) << ',' << goal(2) << ',';
        if (hit == nullptr)
        {
            row << ",,,,,,,,";
        }
        else
        {
            const double ex = hit->x - tcp(0);
            const double ey = hit->y - tcp(1);
            const double ez = hit->z - tcp(2);
            row << hit->index << ',' << hit->class_id << ','
                << hit->x << ',' << hit->y << ',' << hit->z << ','
                << ex << ',' << ey << ',' << ez << ','
                << std::hypot(ex, ey);
        }
        append_csv(path, header, row.str());
    };

    std::cout << std::fixed << std::setprecision(4)
              << "[手眼] " << hand_name(hand) << ' ' << tag
              << (rewrite ? "  修正开" : "  修正已关")
              << "  指尖 (" << tcp(0) << ", " << tcp(1) << ", " << tcp(2)
              << ") m  头相机目标 (" << goal(0) << ", " << goal(1) << ", " << goal(2)
              << ") m  " << noun << "数 " << n << '\n';
    if (n == 0)
    {
        std::cout << "[手眼] " << hand_name(hand) << "  这一帧没有" << noun << '\n';
        write_row(nullptr);
        std::cout << std::flush;
        return;
    }
    for (const HoleHit &hit : hits)
    {
        const double ex_cm = (hit.x - tcp(0)) * 100.0;
        const double ey_cm = (hit.y - tcp(1)) * 100.0;
        std::cout << std::setprecision(0)
                  << "[手眼] " << hand_name(hand)
                  << "  检测" << hit.index << " 类" << hit.class_id
                  << std::setprecision(4)
                  << "  基座 (" << hit.x << ", " << hit.y << ", " << hit.z << ") m"
                  << std::setprecision(2)
                  << "  误差 " << ex_cm << " cm, " << ey_cm
                  << " cm  距离 " << (hit.d_tcp * 100.0) << " cm\n";
        write_row(&hit);
    }
    if (n > 1)
        std::cout << "[手眼] " << hand_name(hand)
                  << "  这一帧有 " << n << " 个" << noun
                  << "，只锁定离指尖 3 cm 内的那个\n";
    std::cout << std::flush;
}

struct JpegJob
{
    cv::Mat bgr;
    std::filesystem::path path;
    std::string note;
};

std::mutex g_jpeg_mu;
std::condition_variable g_jpeg_cv;
std::deque<JpegJob> g_jpeg_q;
std::atomic<bool> g_jpeg_run{true};
std::once_flag g_jpeg_once;

void jpeg_loop()
{
    while (true)
    {
        JpegJob job;
        {
            std::unique_lock<std::mutex> lock(g_jpeg_mu);
            g_jpeg_cv.wait(lock, [] {
                return !g_jpeg_q.empty() || !g_jpeg_run.load(std::memory_order_acquire);
            });
            if (g_jpeg_q.empty())
                return;
            job = std::move(g_jpeg_q.front());
            g_jpeg_q.pop_front();
        }
        std::error_code ec;
        std::filesystem::create_directories(job.path.parent_path(), ec);
        if (!cv::imwrite(job.path.string(), job.bgr))
            std::cerr << "[手眼图] 保存失败 " << job.path.string() << '\n';
        else if (!job.note.empty())
            std::cout << job.note << '\n' << std::flush;
    }
}

void enqueue_jpeg(cv::Mat bgr, std::filesystem::path path, std::string note)
{
    std::call_once(g_jpeg_once, [] {
        std::thread(jpeg_loop).detach();
    });
    {
        std::lock_guard<std::mutex> lock(g_jpeg_mu);
        if (g_jpeg_q.size() >= 4)
            g_jpeg_q.pop_front();
        g_jpeg_q.push_back(JpegJob{std::move(bgr), std::move(path), std::move(note)});
    }
    g_jpeg_cv.notify_one();
}

void save_descent_view(
    int hand,
    const CameraFrameData &frame,
    const std::vector<PoseTargetResult> &targets,
    int want_class,
    const HoleHit *chosen,
    const Eigen::Matrix<double, 1, 6> &tcp,
    bool have_tcp,
    bool accepting,
    bool locked,
    bool held,
    double nearest_tcp_m)
{
    if (!frame.ok || frame.width <= 0 || frame.height <= 0 || frame.color_bgr.empty())
        return;
    cv::Mat bgr(frame.height, frame.width, CV_8UC3,
                const_cast<unsigned char *>(frame.color_bgr.data()));
    cv::Mat canvas = bgr.clone();
    const char *noun = target_noun(want_class);
    for (const PoseTargetResult &target : targets)
    {
        const bool wanted = target.class_id == want_class;
        const bool is_chosen = chosen != nullptr && target.success &&
                               std::abs(target.box_u - chosen->box_u) < 0.5f &&
                               std::abs(target.box_v - chosen->box_v) < 0.5f;
        const cv::Scalar color = is_chosen ? cv::Scalar(0, 220, 0)
                                           : (wanted ? cv::Scalar(0, 220, 220)
                                                     : cv::Scalar(160, 160, 160));
        const int thick = is_chosen ? 3 : 1;
        if (target.box_x1 > target.box_x0 && target.box_y1 > target.box_y0)
        {
            cv::rectangle(canvas,
                          cv::Point(static_cast<int>(target.box_x0), static_cast<int>(target.box_y0)),
                          cv::Point(static_cast<int>(target.box_x1), static_cast<int>(target.box_y1)),
                          color, thick);
        }
        else if (target.box_u >= 0.f && target.box_v >= 0.f)
        {
            cv::circle(canvas,
                       cv::Point(static_cast<int>(target.box_u), static_cast<int>(target.box_v)),
                       10, color, thick);
        }
        if (target.box_u < 0.f || target.box_v < 0.f)
            continue;
        cv::drawMarker(canvas,
                       cv::Point(static_cast<int>(target.box_u), static_cast<int>(target.box_v)),
                       color, cv::MARKER_CROSS, is_chosen ? 22 : 14, is_chosen ? 2 : 1);
    }

    const float grip_u = hand == 1 ? g_move_cfg.cameras.right_gripper_u
                                   : g_move_cfg.cameras.left_gripper_u;
    const float grip_v = hand == 1 ? g_move_cfg.cameras.right_gripper_v
                                   : g_move_cfg.cameras.left_gripper_v;
    const cv::Point grip(static_cast<int>(std::lround(grip_u)),
                         static_cast<int>(std::lround(grip_v)));
    cv::drawMarker(canvas, grip, cv::Scalar(255, 0, 255), cv::MARKER_CROSS, 36, 2);
    cv::circle(canvas, grip, 8, cv::Scalar(255, 0, 255), 1);
    cv::putText(canvas, "gripper", grip + cv::Point(12, -12),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);

    std::ostringstream banner;
    banner << std::fixed << (want_class == 1 ? "PLACE hole" : "GRASP workpiece");
    if (!accepting)
        banner << "  approaching, not used";
    else if (held)
        banner << "  HOLD last target, this frame missed";
    else if (chosen == nullptr)
    {
        banner << "  NO LOCK";
        if (nearest_tcp_m >= 0.0)
            banner << std::setprecision(1) << " nearest " << (nearest_tcp_m * 100.0) << "cm";
    }
    else if (have_tcp)
    {
        if (std::isfinite(chosen->x) && std::isfinite(chosen->y))
        {
            banner << std::setprecision(1)
                   << "  LOCK dx=" << ((chosen->x - tcp(0)) * 100.0)
                   << "cm dy=" << ((chosen->y - tcp(1)) * 100.0) << "cm";
        }
        if (chosen->box_u >= 0.f)
        {
            const float grip_u = hand == 1 ? g_move_cfg.cameras.right_gripper_u
                                           : g_move_cfg.cameras.left_gripper_u;
            const float grip_v = hand == 1 ? g_move_cfg.cameras.right_gripper_v
                                           : g_move_cfg.cameras.left_gripper_v;
            banner << "  px " << std::setprecision(0)
                   << (chosen->box_u - grip_u) << "," << (chosen->box_v - grip_v);
        }
    }
    (void)locked;
    cv::putText(canvas, banner.str(), cv::Point(12, 32),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
    cv::putText(canvas, "GREEN=locked  YELLOW=same class  GRAY=ignored  PURPLE=gripper",
                cv::Point(12, 62), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 1);

    const std::string stamp = now_text();
    std::string file_stamp;
    for (char ch : stamp)
    {
        if (ch == '-' || ch == ':' || ch == ' ' || ch == '.')
            file_stamp.push_back('_');
        else
            file_stamp.push_back(ch);
    }
    const char *folder = hand == 1 ? "right_hand" : "left_hand";
    const std::filesystem::path path =
        std::filesystem::path(project_root_dir()) / "picture_debug" / folder /
        (std::string("下降_") + file_stamp + ".jpg");
    std::ostringstream note;
    note << "[手眼图] " << hand_name(hand) << ' ' << path.string()
         << "  绿框=已锁定的" << noun
         << "，黄框=同类，灰框=忽略，紫十字=夹爪中点";
    enqueue_jpeg(std::move(canvas), path, note.str());
}

void save_raw_descent_frame(const CameraFrameData &frame)
{
    if (!g_raw_descent.load(std::memory_order_acquire))
        return;
    if (!frame.ok || frame.width <= 0 || frame.height <= 0 || frame.color_bgr.empty())
        return;
    const cv::Mat view(frame.height, frame.width, CV_8UC3,
                       const_cast<unsigned char *>(frame.color_bgr.data()));
    cv::Mat bgr = view.clone();
    const std::string stamp = now_text();
    std::string file_stamp;
    for (char ch : stamp)
    {
        if (ch == '-' || ch == ':' || ch == ' ' || ch == '.')
            file_stamp.push_back('_');
        else
            file_stamp.push_back(ch);
    }
    const int seq = g_raw_seq.fetch_add(1, std::memory_order_relaxed) + 1;
    std::ostringstream name;
    name << file_stamp << '_' << std::setw(4) << std::setfill('0') << seq << ".jpg";
    const std::filesystem::path path =
        std::filesystem::path(project_root_dir()) / "hand_descent_raw" / name.str();
    enqueue_jpeg(std::move(bgr), path, std::string("[采图] ") + path.string());
}

struct LockResult
{
    bool locked = false;
    bool first = false;
    bool held = false;
    const HoleHit *matched = nullptr;
    double gap_m = -1.0;
};

constexpr double kLockAcquireM = 0.03;
constexpr double kLockTrackM = 0.02;
/** 框中心离夹爪像素在这个范围内，即使深度点飞了也锁定。 */
constexpr double kLockPixelPx = 110.0;
constexpr double kTrackPixelPx = 160.0;

LockResult update_target_lock(int hand, const std::vector<HoleHit> &hits)
{
    LockResult out;
    std::lock_guard<std::mutex> lock(g_slot[hand].mu);
    Slot &slot = g_slot[hand];
    if (!slot.active)
        return out;

    const float grip_u = hand == 1 ? g_move_cfg.cameras.right_gripper_u
                                   : g_move_cfg.cameras.left_gripper_u;
    const float grip_v = hand == 1 ? g_move_cfg.cameras.right_gripper_v
                                   : g_move_cfg.cameras.left_gripper_v;
    auto pixel_gap = [&](const HoleHit &hit) {
        if (hit.box_u < 0.f || hit.box_v < 0.f)
            return 1.0e9;
        return std::hypot(static_cast<double>(hit.box_u) - grip_u,
                          static_cast<double>(hit.box_v) - grip_v);
    };
    auto remember = [&](const HoleHit &hit) {
        slot.hole_locked = true;
        slot.lock_x = hit.x;
        slot.lock_y = hit.y;
        slot.lock_u = hit.box_u;
        slot.lock_v = hit.box_v;
        slot.lock_index = hit.index;
    };

    const HoleHit *nearest = nullptr;
    double nearest_px = 1.0e9;
    for (const HoleHit &hit : hits)
    {
        const double px = pixel_gap(hit);
        if (px < nearest_px)
        {
            nearest_px = px;
            nearest = &hit;
        }
    }

    const HoleHit *tracked = nullptr;
    double tracked_gap = -1.0;
    if (slot.hole_locked)
    {
        double best_box = 1.0e9;
        for (const HoleHit &hit : hits)
        {
            const double d3 = std::hypot(hit.x - slot.lock_x, hit.y - slot.lock_y);
            const double dpx = (slot.lock_u < 0.f || hit.box_u < 0.f)
                                   ? 1.0e9
                                   : std::hypot(static_cast<double>(hit.box_u - slot.lock_u),
                                                static_cast<double>(hit.box_v - slot.lock_v));
            const bool by_depth = std::isfinite(d3) && d3 <= kLockTrackM;
            const bool by_pixel = dpx <= kTrackPixelPx;
            if (!by_depth && !by_pixel)
                continue;
            if (dpx < best_box)
            {
                best_box = dpx;
                tracked_gap = d3;
                tracked = &hit;
            }
        }
    }

    const bool near_gripper = nearest != nullptr && nearest_px <= kLockPixelPx;
    const double tracked_to_grip = tracked != nullptr ? pixel_gap(*tracked) : 1.0e9;
    const bool closer =
        near_gripper && (tracked == nullptr || nearest_px + 40.0 < tracked_to_grip);

    if (!slot.hole_locked)
    {
        if (near_gripper)
        {
            remember(*nearest);
            out.matched = nearest;
            out.first = true;
            out.gap_m = nearest_px;
        }
        else if (nearest != nullptr)
            out.gap_m = nearest_px;
    }
    else if (closer && nearest != tracked)
    {
        remember(*nearest);
        out.matched = nearest;
        out.first = true;
        out.gap_m = nearest_px;
        std::cout << std::fixed << std::setprecision(0)
                  << "[锁定] " << hand_name(hand)
                  << " 换到离夹爪更近的工件，距离 " << nearest_px << " px\n"
                  << std::flush;
    }
    else if (tracked != nullptr)
    {
        remember(*tracked);
        out.matched = tracked;
        out.gap_m = tracked_gap;
    }
    else
    {
        out.held = true;
        out.gap_m = nearest == nullptr ? -1.0 : nearest_px;
    }
    out.locked = slot.hole_locked;
    return out;
}

bool command_locked_hit(
    int hand,
    Robot_Arm &arm,
    const double q[7],
    const CameraFrameData &frame,
    const std::array<double, 16> &eye_T,
    const Eigen::Matrix<double, 1, 6> &goal,
    const HoleHit &hit,
    double gain,
    double max_step_m)
{
    if (!g_move_cfg.cameras.hand_descent_correct || !descent_player_accepting(hand))
        return false;
    if (hit.box_u < 0.f || hit.box_v < 0.f)
        return false;
    const float grip_u = hand == 1 ? g_move_cfg.cameras.right_gripper_u
                                   : g_move_cfg.cameras.left_gripper_u;
    const float grip_v = hand == 1 ? g_move_cfg.cameras.right_gripper_v
                                   : g_move_cfg.cameras.left_gripper_v;
    const double du = static_cast<double>(hit.box_u) - grip_u;
    const double dv = static_cast<double>(hit.box_v) - grip_v;
    const char *name = hand_name(hand);
    Eigen::RowVectorXd qv(7);
    for (int i = 0; i < 7; ++i)
        qv(i) = q[i];
    const Eigen::Matrix<double, 1, 6> tcp = arm_tcp_from_joints(arm, qv);
    // 夹爪开合是图像的水平方向。u 对准即可；上下大约 10 px 以上也不再推。
    constexpr double kDeadUPx = 8.0;
    descent_note_align_px(hand, std::abs(du));
    if (std::abs(du) <= kDeadUPx)
    {
        std::cout << std::fixed << std::setprecision(0)
                  << "[图像] " << name << " 水平差 " << du
                  << " px，上下差 " << dv << " px，物块已在夹爪中间，保持\n"
                  << std::flush;
        descent_apply_hole_xy(hand, tcp(0), tcp(1), "已对齐");
        return true;
    }
    const double fx = frame.K[0];
    const double fy = frame.K[4];
    if (!(fx > 1.0) || !(fy > 1.0))
        return false;

    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    Eigen::Vector3d axis_x = Eigen::Vector3d::UnitX();
    Eigen::Vector3d axis_y = Eigen::Vector3d::UnitY();
    Eigen::Vector3d axis_z = Eigen::Vector3d::UnitZ();
    if (!arm_camera_axes_in_base(arm, qv, eye_T, origin, axis_x, axis_y, axis_z))
    {
        std::cout << "[图像] " << name << " 手相机姿态无效，这一帧不改\n" << std::flush;
        return false;
    }
    double dist_m = 0.08;
    if (std::abs(axis_z.z()) > 0.3)
        dist_m = (goal(2) - origin.z()) / axis_z.z();
    if (!std::isfinite(dist_m))
        dist_m = 0.08;
    if (dist_m < 0.04)
        dist_m = 0.04;
    if (dist_m > 0.18)
        dist_m = 0.18;

    (void)axis_y;
    (void)fy;
    const Eigen::Vector3d delta = axis_x * (du * dist_m / fx);
    if (!std::isfinite(gain) || gain < 0.0)
        gain = 0.0;
    if (gain > 1.0)
        gain = 1.0;
    const double cap = (std::isfinite(max_step_m) && max_step_m > 0.0) ? max_step_m : 0.012;
    double dx = delta.x() * gain;
    double dy = delta.y() * gain;
    const double step = std::hypot(dx, dy);
    if (step > cap && step > 1e-9)
    {
        dx *= cap / step;
        dy *= cap / step;
    }
    std::ostringstream why;
    why << std::fixed << std::setprecision(0) << "水平差 " << du << " px，上下差 " << dv << " px";
    std::cout << std::fixed << std::setprecision(0)
              << "[图像] " << name << " 水平差 " << du << " px，上下差 " << dv
              << " px，只改水平，这一帧最多 "
              << std::setprecision(1) << (std::hypot(dx, dy) * 1000.0) << " mm\n"
              << std::flush;
    descent_apply_hole_xy(hand, tcp(0) + dx, tcp(1) + dy, why.str().c_str());
    return true;
}

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
    const bool grabbed_in_descent = frame.ok && descent_player_accepting(hand);
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
    double issue_ms = -1.0;
    if (frame.ok && g_run.load(std::memory_order_acquire))
    {
        const auto infer_t0 = std::chrono::steady_clock::now();
        try
        {
            std::lock_guard<std::mutex> infer_lock(g_infer_mu);
            result = bridge->run_yolo(
                frame, algorithm_id_for_slot(hand_slot(hand)), false, hand_slot(hand), -1,
                SegEngineId::Hand);
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
    std::vector<HoleHit> hits;
    std::ostringstream candidates;
    int n_det = 0;
    bool have_tcp = false;
    int want_class = -1;
    Robot_Arm *arm = nullptr;
    Eigen::Matrix<double, 1, 6> tcp = Eigen::Matrix<double, 1, 6>::Zero();
    std::string skip;
    double best_rank = 1e9;
    double color_best_px = 1e9;

    {
        std::lock_guard<std::mutex> lock(slot.mu);
        if (!slot.active || slot.gen != gen || slot.arm == nullptr)
            skip = "伸手已结束";
        else if (!have_q)
            skip = "还没有指令关节";
        else
        {
            want_class = slot.want_class;
            arm = slot.arm;
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
                        if (frame.depth_m.empty())
                        {
                            if (target.box_u < 0.f || target.box_v < 0.f)
                            {
                                candidates << target.class_id << ':' << std::fixed
                                           << std::setprecision(2) << target.confidence
                                           << ":无框;";
                                continue;
                            }
                            const bool wanted = want_class < 0
                                                    ? (target.class_id == 0 || target.class_id == 1)
                                                    : (target.class_id == want_class);
                            candidates << target.class_id << ':' << std::fixed
                                       << std::setprecision(2) << target.confidence << ":框";
                            if (want_class >= 0 && !wanted)
                                candidates << ":忽略";
                            candidates << ';';
                            if (!wanted)
                                continue;
                            const float grip_u = hand == 1 ? g_move_cfg.cameras.right_gripper_u
                                                           : g_move_cfg.cameras.left_gripper_u;
                            const float grip_v = hand == 1 ? g_move_cfg.cameras.right_gripper_v
                                                           : g_move_cfg.cameras.left_gripper_v;
                            const double px = std::hypot(
                                static_cast<double>(target.box_u) - grip_u,
                                static_cast<double>(target.box_v) - grip_v);
                            HoleHit hit;
                            hit.index = n_det;
                            hit.class_id = target.class_id;
                            hit.x = std::numeric_limits<double>::quiet_NaN();
                            hit.y = hit.x;
                            hit.z = hit.x;
                            hit.d_tcp = 1.0e9;
                            hit.d_goal = 0.0;
                            hit.confidence = target.confidence;
                            hit.box_u = target.box_u;
                            hit.box_v = target.box_v;
                            hit.box_x0 = target.box_x0;
                            hit.box_y0 = target.box_y0;
                            hit.box_x1 = target.box_x1;
                            hit.box_y1 = target.box_y1;
                            hits.push_back(hit);
                            if (px < color_best_px)
                            {
                                color_best_px = px;
                                best.class_id = target.class_id;
                                best.class_name = target.class_name;
                                best.confidence = target.confidence;
                                best.success = true;
                                best.box_u = target.box_u;
                                best.box_v = target.box_v;
                                best.message = "彩色框";
                            }
                            continue;
                        }
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
                        const bool wanted = want_class < 0
                                                ? (target.class_id == 0 || target.class_id == 1)
                                                : (target.class_id == want_class);
                        const double d_tcp = have_tcp
                                                 ? std::hypot(base(0) - tcp(0), base(1) - tcp(1))
                                                 : dxy;
                        if (wanted && have_tcp)
                        {
                            HoleHit hit;
                            hit.index = n_det;
                            hit.class_id = target.class_id;
                            hit.x = base(0);
                            hit.y = base(1);
                            hit.z = base(2);
                            hit.d_tcp = d_tcp;
                            hit.d_goal = dxy;
                            hit.confidence = target.confidence;
                            hit.box_u = target.box_u;
                            hit.box_v = target.box_v;
                            hit.box_x0 = target.box_x0;
                            hit.box_y0 = target.box_y0;
                            hit.box_x1 = target.box_x1;
                            hit.box_y1 = target.box_y1;
                            hits.push_back(hit);
                        }
                        candidates << target.class_id << ':' << std::fixed << std::setprecision(2)
                                   << target.confidence << ':' << std::setprecision(1)
                                   << (dxy * 1000.0) << "mm";
                        if (want_class >= 0 && !wanted)
                            candidates << ":忽略";
                        candidates << ';';
                        const double rank = want_class >= 0 ? (wanted && have_tcp ? d_tcp : -1.0) : dxy;
                        if (rank >= 0.0 && (!best.have || rank < best_rank))
                        {
                            best_rank = rank;
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
    {
        if (grabbed_in_descent)
            save_raw_descent_frame(frame);
        return;
    }

    const bool pixel_box = frame.depth_m.empty() && best.box_u >= 0.f;
    const bool accepting = have_tcp && descent_player_accepting(hand);
    LockResult lock;
    if (accepting && want_class >= 0)
    {
        lock = update_target_lock(hand, hits);
        int shots = 0;
        double shot_progress = -1.0;
        {
            std::lock_guard<std::mutex> lock_slot(slot.mu);
            shots = slot.xy_shots;
            shot_progress = slot.xy_shot_progress;
        }
        const double progress = descent_vertical_progress(hand);
        const bool first_window = shots == 0 && progress >= 0.35;
        const bool second_window =
            shots == 1 && progress >= std::max(0.70, shot_progress + 0.15);
        const bool window = first_window || second_window;
        const double gain = shots == 0 ? 0.55 : 0.50;
        const double cap_m = shots == 0 ? 0.012 : 0.006;
        if (shots >= 2)
        {
            descent_note_align_px(hand, -1.0);
        }
        else if (window && lock.matched != nullptr && arm != nullptr &&
                 command_locked_hit(
                     hand, *arm, q, frame, eye.T, goal, *lock.matched, gain, cap_m))
        {
            {
                std::lock_guard<std::mutex> lock_slot(slot.mu);
                if (slot.xy_shots == 0)
                    slot.xy_shot_progress = progress;
                slot.xy_shots += 1;
            }
            issue_ms = std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - grab_t0)
                           .count();
            std::cout << std::fixed << std::setprecision(0)
                      << "[图像] " << hand_name(hand) << " 下降 "
                      << (progress * 100.0) << "% 第 " << (shots + 1)
                      << " 次修正水平"
                      << (shots == 0 ? "，70% 再看一次\n" : "，之后只监控\n")
                      << std::flush;
        }
        else if (window)
        {
            descent_note_align_px(hand, -1.0);
            std::cout << std::fixed << std::setprecision(0)
                      << "[图像] " << hand_name(hand) << " 下降 "
                      << (progress * 100.0) << "% 这帧没锁定工件，下一帧再试\n"
                      << std::flush;
        }
        else
        {
            descent_note_align_px(hand, -1.0);
        }
    }

    if (!result.ok && skip.empty())
        skip = result.message.empty() ? "推理失败" : result.message;
    else if (!eye.ok && skip.empty() && !pixel_box)
        skip = eye.err.empty() ? "手眼读取失败" : eye.err;
    else if (result.ok && n_det == 0 && skip.empty())
        skip = "未检测到目标";
    else if (result.ok && !best.have && !pixel_box && skip.empty())
        skip = want_class == 0 ? "没有工件" : (want_class == 1 ? "没有孔" : "没有成功的位姿");

    const bool accept = best.have && skip.empty() && best.dxy_m <= match_m;
    std::string verdict = "无观测";
    std::string reason = skip;
    if (pixel_box && skip.empty())
        verdict = "像素";
    else if (best.have && skip.empty())
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
    if (pixel_box)
    {
        const float grip_u = hand == 1 ? g_move_cfg.cameras.right_gripper_u
                                       : g_move_cfg.cameras.left_gripper_u;
        const float grip_v = hand == 1 ? g_move_cfg.cameras.right_gripper_v
                                       : g_move_cfg.cameras.left_gripper_v;
        std::cout << "[手相机] " << hand_name(hand) << ' ' << tag
                  << "  彩色框 " << std::setprecision(0) << best.box_u << "," << best.box_v
                  << "  离夹爪 " << (best.box_u - grip_u) << "," << (best.box_v - grip_v)
                  << " px\n";
    }
    else if (best.have)
    {
        std::cout << "[手相机] " << hand_name(hand) << ' ' << tag
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
        std::cout << "[手相机] " << hand_name(hand) << ' ' << tag
                  << "  无有效目标";
        if (!reason.empty())
            std::cout << "  " << reason;
        std::cout << "  检测数 " << n_det << '\n';
    }
    std::cout << "[手相机] " << hand_name(hand) << ' ' << tag
              << "  推理 " << std::setprecision(0) << infer_ms
              << " ms  取帧 " << grab_ms << " ms";
    if (issue_ms >= 0.0)
        std::cout << "  到下发 " << std::setprecision(0) << issue_ms << " ms";
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
    if (grabbed_in_descent)
        save_raw_descent_frame(frame);
    const char *noun = target_noun(want_class);
    if (accepting && want_class >= 0)
    {
        std::cout << std::fixed << std::setprecision(1) << "[锁定] " << hand_name(hand) << ' ';
        if (lock.first && lock.matched != nullptr)
        {
            std::cout << "锁定" << noun;
            if (std::isfinite(lock.matched->x))
                std::cout << "，离指尖 " << (lock.gap_m * 100.0) << " cm";
            else
                std::cout << "，按彩色框";
            std::cout << '\n';
        }
        else if (lock.matched != nullptr)
        {
            std::cout << "仍是这个" << noun;
            if (std::isfinite(lock.matched->x))
                std::cout << "，离锁定点 " << (lock.gap_m * 100.0) << " cm";
            else
                std::cout << "，按彩色框";
            std::cout << '\n';
        }
        else if (lock.held)
            std::cout << "这一帧没对上锁定" << noun << "，保持上次坐标\n";
        else if (lock.gap_m >= 0.0 && std::isfinite(lock.gap_m) && lock.gap_m < 10.0)
            std::cout << "最近" << noun << "离指尖 " << (lock.gap_m * 100.0)
                      << " cm，超过 3 cm，不锁定\n";
        else
            std::cout << "这一帧没有" << noun << "，不锁定\n";
        std::cout << std::flush;
    }
    if (want_class >= 0)
        save_descent_view(hand, frame, result.targets, want_class, lock.matched, tcp, have_tcp,
                          accepting, lock.locked, lock.held, lock.gap_m);
    if (accepting)
        log_hand_eye_bias(hand, tag, goal, tcp, hits, want_class);
}

void save_track_preview(int hand, const CameraFrameData &frame)
{
    if (hand < 0 || hand > 1 || !frame.ok || frame.width <= 0 || frame.height <= 0 ||
        frame.color_bgr.empty())
        return;
    static std::mutex preview_mu;
    static std::chrono::steady_clock::time_point last[2]{};
    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(preview_mu);
        if (last[hand].time_since_epoch().count() != 0 &&
            now - last[hand] < std::chrono::seconds(2))
            return;
        last[hand] = now;
    }
    const cv::Mat bgr(frame.height, frame.width, CV_8UC3,
                      const_cast<unsigned char *>(frame.color_bgr.data()));
    const char *file = hand == 1 ? "track_right.jpg" : "track_left.jpg";
    const std::filesystem::path path =
        std::filesystem::path(project_root_dir()) / "picture_debug" / file;
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
    cv::imwrite(path.string(), bgr);
}

bool sample_nearest_workpiece(
    int hand,
    Robot_Arm &arm,
    SegPoseBridge &bridge,
    double &x_m,
    double &y_m,
    double &tcp_gap_m)
{
    tcp_gap_m = -1.0;
    const char *name = hand_name(hand);
    auto note = [&](const char *result, const std::string &detail) {
        std::cout << "[跟随] " << name << ' ' << result;
        if (!detail.empty())
            std::cout << ' ' << detail;
        std::cout << '\n' << std::flush;
        hand_track_log(name, result, detail);
    };
    if (hand < 0 || hand > 1)
        return false;
    if (RealSenseMultiCam::live() == nullptr)
    {
        note("手相机未启动", "");
        return false;
    }

    CameraFrameData frame = RealSenseMultiCam::live()->grab(hand_slot(hand));
    if (!frame.ok)
    {
        note("取帧失败", frame.message);
        return false;
    }
    if (image_rotation_deg(hand) == 180)
        rotate_frame_180(frame);
    save_track_preview(hand, frame);

    double q_cmd[7] = {};
    double q_age_ms = -1.0;
    Eigen::RowVectorXd qv(7);
    const bool used_cmd = read_q(hand, q_cmd, q_age_ms);
    if (used_cmd)
    {
        for (int i = 0; i < 7; ++i)
            qv(i) = q_cmd[i];
    }
    else
    {
        const Eigen::Matrix<double, 1, 7> q7 = arm_get_joint_pos(arm);
        for (int i = 0; i < 7; ++i)
            qv(i) = q7(i);
    }
    const char *q_src = used_cmd ? "关节用指令角" : "关节用编码器";

    PoseRunResult result;
    try
    {
        std::lock_guard<std::mutex> infer_lock(g_infer_mu);
        result = bridge.run_yolo(
            frame, algorithm_id_for_slot(hand_slot(hand)), false, hand_slot(hand), -1,
            SegEngineId::Hand);
    }
    catch (const std::exception &ex)
    {
        note("推理失败", ex.what());
        return false;
    }
    if (!result.ok)
    {
        note("推理失败", result.message);
        return false;
    }

    ensure_eye();
    Eye eye;
    {
        std::lock_guard<std::mutex> lock(g_eye_mu);
        eye = g_eye[hand];
    }
    if (!eye.ok)
    {
        note("手眼未读到", "");
        return false;
    }

    Eigen::Matrix<double, 1, 6> tcp = Eigen::Matrix<double, 1, 6>::Zero();
    bool have_tcp = false;
    try
    {
        tcp = arm_tcp_from_joints(arm, qv);
        have_tcp = tcp.allFinite();
    }
    catch (const std::exception &ex)
    {
        note("指尖正解失败", ex.what());
        return false;
    }
    if (!have_tcp)
    {
        note("指尖正解失败", q_src);
        return false;
    }

    const HoleHit *best = nullptr;
    HoleHit chosen;
    double best_d = 1e9;
    int n_work = 0;
    for (const PoseTargetResult &target : result.targets)
    {
        if (!target.success || target.class_id != 0)
            continue;
        ++n_work;
        try
        {
            const std::array<double, 16> flange =
                transform_pose_cam_to_robot(eye.T, target.pose_4x4);
            Eigen::Matrix<double, 1, 6> flange_pose =
                T2PosEulerAngles(row_major_matrix(flange));
            const Eigen::Matrix<double, 1, 6> base =
                arm_base_visual_from_joints(arm, qv, flange_pose);
            if (!base.head<3>().allFinite())
                continue;
            const double d = std::hypot(base(0) - tcp(0), base(1) - tcp(1));
            if (best == nullptr || d < best_d)
            {
                best_d = d;
                chosen.x = base(0);
                chosen.y = base(1);
                chosen.z = base(2);
                chosen.d_tcp = d;
                best = &chosen;
            }
        }
        catch (const std::exception &)
        {
        }
    }
    if (best == nullptr)
    {
        note("没有可用工件", n_work == 0 ? "这一帧没框到工件，XY 保持" : "框到了但点无效，XY 保持");
        return false;
    }
    x_m = best->x;
    y_m = best->y;
    tcp_gap_m = best->d_tcp;
    std::ostringstream detail;
    detail << std::fixed << std::setprecision(4) << "基座 (" << x_m << ", " << y_m
           << ") 离指尖 " << std::setprecision(1) << (tcp_gap_m * 100.0)
           << " cm 高度差 " << ((chosen.z - tcp(2)) * 100.0) << " cm " << q_src;
    note("工件", detail.str());
    return true;
}

void shadow_loop(int hand)
{
    while (g_run.load(std::memory_order_acquire))
    {
        bool active = false;
        {
            std::lock_guard<std::mutex> lock(g_slot[hand].mu);
            active = g_slot[hand].active;
        }
        if (hardware_abort_requested() || !active)
        {
            std::unique_lock<std::mutex> lock(g_wait_mu);
            g_cv.wait_for(lock, std::chrono::milliseconds(30), [] {
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
            std::cerr << "[手相机] 观测失败: " << ex.what() << '\n';
        }
    }
}

} // namespace

void start_shadow_thread()
{
    std::lock_guard<std::mutex> lock(g_start_mu);
    if (g_bridge.load(std::memory_order_acquire) == nullptr)
        return;
    g_run.store(true, std::memory_order_release);
    for (int hand = 0; hand < 2; ++hand)
    {
        if (!g_threads[hand].joinable())
            g_threads[hand] = std::thread(shadow_loop, hand);
    }
}

void hand_shadow_bind(SegPoseBridge *bridge)
{
    if (bridge == nullptr)
    {
        g_watch.store(false, std::memory_order_release);
        g_run.store(false, std::memory_order_release);
        g_cv.notify_all();
        for (int hand = 0; hand < 2; ++hand)
        {
            if (g_threads[hand].joinable())
                g_threads[hand].join();
        }
        g_bridge.store(nullptr, std::memory_order_release);
        return;
    }
    g_bridge.store(bridge, std::memory_order_release);
    std::cout << "[手相机] 平时不取帧。竖直下降时取帧";
    if (g_move_cfg.cameras.hand_descent_correct)
        std::cout << "，锁定后把框中心对齐夹爪像素\n";
    else
        std::cout << "，只记录，不改轨迹。画面写入 picture_debug/left_hand 和 right_hand\n";
    std::cout << std::flush;
}

bool hand_track_nearest_workpiece(
    int hand,
    Robot_Arm &arm,
    SegPoseBridge &bridge,
    double &x_m,
    double &y_m,
    double &tcp_gap_m)
{
    return sample_nearest_workpiece(hand, arm, bridge, x_m, y_m, tcp_gap_m);
}

void hand_shadow_set_descent_raw(bool enable)
{
    g_raw_descent.store(enable, std::memory_order_release);
    if (!enable)
        return;
    const std::filesystem::path dir =
        std::filesystem::path(project_root_dir()) / "hand_descent_raw";
    std::cout << "[采图] 只在竖直下降段保存原图，目录 " << dir.string() << '\n' << std::flush;
}

void hand_track_log(const std::string &hand, const std::string &result, const std::string &detail)
{
    std::string clean = detail;
    for (char &ch : clean)
    {
        if (ch == ',' || ch == '\n' || ch == '\r')
            ch = ' ';
    }
    std::ostringstream line;
    line << now_text() << ',' << hand << ',' << result << ',' << clean;
    append_csv(
        std::filesystem::path(project_root_dir()) / "logs" / "hand_track.csv",
        "时间,手,结果,说明\n",
        line.str());
}

void hand_shadow_note_q(int hand, const double q_rad[7])
{
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

void hand_shadow_ensure(
    int hand,
    Robot_Arm *arm,
    const Eigen::Matrix<double, 1, 6> &goal,
    const char *tag,
    int want_class)
{
    g_watch.store(true, std::memory_order_release);
    start_shadow_thread();
    hand_shadow_begin(hand, arm, goal, tag, want_class);
}

void hand_shadow_begin(
    int hand,
    Robot_Arm *arm,
    const Eigen::Matrix<double, 1, 6> &goal,
    const char *tag,
    int want_class)
{
    if (!g_run.load(std::memory_order_acquire))
        return;
    if (!g_watch.load(std::memory_order_acquire))
        return;
    if (hand < 0 || hand > 1 || arm == nullptr)
        return;
    if (g_bridge.load(std::memory_order_acquire) == nullptr)
    {
        std::cout << "[手相机] 算法引擎还没绑定，这次伸手不观测\n" << std::flush;
        return;
    }
    {
        std::lock_guard<std::mutex> lock(g_slot[hand].mu);
        g_slot[hand].arm = arm;
        g_slot[hand].goal = goal;
        g_slot[hand].tag = tag != nullptr ? tag : "";
        g_slot[hand].want_class = want_class;
        g_slot[hand].hole_locked = false;
        g_slot[hand].lock_x = 0.0;
        g_slot[hand].lock_y = 0.0;
        g_slot[hand].lock_index = -1;
        g_slot[hand].xy_shots = 0;
        g_slot[hand].xy_shot_progress = -1.0;
        g_slot[hand].gen += 1u;
        g_slot[hand].active = true;
    }
    const bool rewrite = g_move_cfg.cameras.hand_descent_correct &&
                         g_watch.load(std::memory_order_acquire);
    std::cout << std::fixed << std::setprecision(4)
              << "[手相机] " << hand_name(hand) << ' ' << (tag != nullptr ? tag : "")
              << " 开始  目标 (" << goal(0) << ", " << goal(1) << ", " << goal(2)
              << ") m  "
              << (rewrite ? "下降到 35% 和 70% 各修正一次水平，之后只监控" : "只记录，不下发")
              << '\n'
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
        g_slot[hand].hole_locked = false;
        g_slot[hand].gen += 1u;
    }
    if (hand == 0 || hand == 1)
    {
        std::lock_guard<std::mutex> lock(g_slot[1 - hand].mu);
        if (!g_slot[1 - hand].active)
            g_watch.store(false, std::memory_order_release);
    }
    if (!was)
        return;
    std::cout << "[手相机] " << hand_name(hand) << ' ' << tag << " 结束\n" << std::flush;
}
