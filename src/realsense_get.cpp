#include "realsense_get.h"

#include "Ti5_socketcan.h"

#include <cmath>
#include <cstring>
#include <iostream>
#include <map>
#include <stdexcept>
#include <thread>
#include <vector>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <pwd.h>
#include <sstream>
#include <unistd.h>

#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

namespace
{

constexpr const char *kSlotKeys[] = {"head", "right_hand", "left_hand"};
constexpr int kWarmupFrames = 8;
constexpr unsigned kGrabWaitTimeoutMs = 5000;
/** 三台同开时 USB 口常首帧超时；启动/重试用更长等待 */
constexpr unsigned kStartWaitTimeoutMs = 12000;
constexpr int kStartMaxAttempts = 4;
constexpr int kStaggerBetweenCamsMs = 400;

struct SlotDevice
{
    std::string slot_id;
    std::string label;
    std::string serial;
    rs2::pipeline pipeline;
    rs2::align align{RS2_STREAM_COLOR};
    float depth_scale = 0.001f;
    std::array<double, 9> K{};
    std::array<double, 5> dist{};
    bool started = false;
};

void try_set_frames_queue_size_one(const rs2::device &device)
{
    for (rs2::sensor sensor : device.query_sensors())
    {
        if (sensor.supports(RS2_OPTION_FRAMES_QUEUE_SIZE))
            sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 1.f);
    }
}

void fill_intrinsics(const rs2_intrinsics &intr, std::array<double, 9> &K, std::array<double, 5> &dist)
{
    K = {
        intr.fx, 0.0, intr.ppx,
        0.0, intr.fy, intr.ppy,
        0.0, 0.0, 1.0};
    for (int i = 0; i < 5; ++i)
        dist[static_cast<size_t>(i)] = intr.coeffs[i];
}

/** 排空 pipeline 内所有已缓冲帧 */
void flush_pipeline(rs2::pipeline &pipeline)
{
    rs2::frameset frames;
    while (pipeline.poll_for_frames(&frames))
    {
    }
}

/** 排空缓冲后连续 wait 两帧，第二帧作为识别用图（停稳后新采集） */
rs2::frameset grab_fresh_frameset(rs2::pipeline &pipeline)
{
    flush_pipeline(pipeline);
    (void)pipeline.wait_for_frames(static_cast<unsigned int>(kGrabWaitTimeoutMs));
    flush_pipeline(pipeline);
    return pipeline.wait_for_frames(static_cast<unsigned int>(kGrabWaitTimeoutMs));
}

/** 排空 pipeline 缓冲，保留 poll 到的最后一帧；若缓冲为空则阻塞等待新帧 */
rs2::frameset grab_newest_frameset(rs2::pipeline &pipeline, unsigned timeout_ms = kGrabWaitTimeoutMs)
{
    rs2::frameset frames;
    rs2::frameset newest;
    while (pipeline.poll_for_frames(&frames))
        newest = std::move(frames);

    if (newest)
        return newest;

    return pipeline.wait_for_frames(timeout_ms);
}

void safe_stop_pipeline(SlotDevice &dev)
{
    try
    {
        dev.pipeline.stop();
    }
    catch (...)
    {
    }
    dev.started = false;
}

void log_connected_realsense()
{
    try
    {
        rs2::context ctx;
        const rs2::device_list list = ctx.query_devices();
        std::cout << "[realsense] 当前已连接 " << list.size() << " 台:\n";
        if (list.size() == 0)
        {
            std::cout << "  (无。检查 USB / 是否被其它进程占用)\n";
            return;
        }
        for (auto &&dev : list)
        {
            const char *name = "unknown";
            const char *sn = "?";
            const char *port = "";
            try
            {
                name = dev.get_info(RS2_CAMERA_INFO_NAME);
            }
            catch (...)
            {
            }
            try
            {
                sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            }
            catch (...)
            {
            }
            try
            {
                port = dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);
            }
            catch (...)
            {
            }
            std::cout << "  - " << name << " SN=" << sn;
            if (port && port[0])
                std::cout << "  port=" << port;
            std::cout << "\n";
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "[realsense] 枚举设备失败: " << e.what() << "\n";
    }
}

/** 对指定 SN 做软件复位（不拔线）；失败忽略 */
void try_hardware_reset_serial(const std::string &serial)
{
    try
    {
        rs2::context ctx;
        for (auto &&dev : ctx.query_devices())
        {
            const std::string sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            if (sn != serial)
                continue;
            std::cout << "[realsense] hardware_reset SN=" << serial << " …\n";
            dev.hardware_reset();
            std::this_thread::sleep_for(std::chrono::milliseconds(2500));
            return;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "[realsense] hardware_reset 失败 SN=" << serial << ": " << e.what() << "\n";
    }
}

bool start_slot_device(SlotDevice &dev, int width, int height, int fps, std::string &err)
{
    err.clear();
    for (int attempt = 1; attempt <= kStartMaxAttempts; ++attempt)
    {
        try
        {
            if (attempt > 1)
            {
                safe_stop_pipeline(dev);
                std::this_thread::sleep_for(std::chrono::milliseconds(500 * attempt));
                if (attempt >= 3)
                    try_hardware_reset_serial(dev.serial);
                // reset 后旧 pipeline 句柄常失效，换新实例
                dev.pipeline = rs2::pipeline();
                dev.align = rs2::align(RS2_STREAM_COLOR);
            }

            rs2::config cfg;
            cfg.enable_device(dev.serial);
            cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
            cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);

            const rs2::pipeline_profile profile = dev.pipeline.start(cfg);
            const rs2::video_stream_profile color_sp =
                profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
            fill_intrinsics(color_sp.get_intrinsics(), dev.K, dev.dist);

            const rs2::device device = profile.get_device();
            const rs2::depth_sensor depth_sensor = device.first<rs2::depth_sensor>();
            dev.depth_scale = depth_sensor.get_depth_scale();
            try_set_frames_queue_size_one(device);

            // 首帧用长超时；USB 同口三台时经常要等 >5s
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            for (int i = 0; i < kWarmupFrames; ++i)
                grab_newest_frameset(dev.pipeline, kStartWaitTimeoutMs);

            dev.started = true;
            if (attempt > 1)
                std::cout << "[realsense] " << dev.label << " 第 " << attempt << " 次启动成功\n";
            return true;
        }
        catch (const std::exception &e)
        {
            err = e.what();
            std::cerr << "[realsense] 打开失败 [" << dev.slot_id << "] SN=" << dev.serial
                      << " 第 " << attempt << "/" << kStartMaxAttempts << ": " << e.what() << "\n";
            safe_stop_pipeline(dev);
        }
    }
    return false;
}

std::string slot_key(CameraSlot slot)
{
    const auto idx = static_cast<size_t>(slot);
    if (idx >= 3)
        return {};
    return kSlotKeys[idx];
}

void mat_bgr_to_vector(const cv::Mat &bgr, std::vector<uint8_t> &out)
{
    CV_Assert(bgr.type() == CV_8UC3 && bgr.isContinuous());
    const size_t n = static_cast<size_t>(bgr.rows) * static_cast<size_t>(bgr.cols) * 3u;
    out.resize(n);
    std::memcpy(out.data(), bgr.data, n);
}

void mat_depth_to_vector(const cv::Mat &depth_m, std::vector<float> &out)
{
    CV_Assert(depth_m.type() == CV_32FC1 && depth_m.isContinuous());
    const size_t n = static_cast<size_t>(depth_m.rows) * static_cast<size_t>(depth_m.cols);
    out.resize(n);
    std::memcpy(out.data(), depth_m.ptr<float>(), n * sizeof(float));
}

cv::Mat depth_z16_to_meters(const rs2::frame &depth_frame, float depth_scale)
{
    const int h = depth_frame.as<rs2::video_frame>().get_height();
    const int w = depth_frame.as<rs2::video_frame>().get_width();
    const auto *raw = reinterpret_cast<const uint16_t *>(depth_frame.get_data());

    cv::Mat depth_m(h, w, CV_32FC1);
    for (int y = 0; y < h; ++y)
    {
        auto *row = depth_m.ptr<float>(y);
        for (int x = 0; x < w; ++x)
        {
            const uint16_t z = raw[y * w + x];
            row[x] = (z == 0) ? std::nanf("") : static_cast<float>(z) * depth_scale;
        }
    }
    return depth_m;
}

} // namespace

struct RealSenseMultiCam::Impl
{
    std::string yaml_path;
    int width = 1280;
    int height = 720;
    int head_fps = 15;
    int hand_fps = 15;
    std::map<std::string, SlotDevice> slots;
};

RealSenseMultiCam::RealSenseMultiCam(
    std::string yaml_path, int width, int height, int head_fps, int hand_fps)
    : impl_(std::make_unique<Impl>())
{
    impl_->yaml_path = std::move(yaml_path);
    impl_->width = width;
    impl_->height = height;
    impl_->head_fps = head_fps;
    impl_->hand_fps = hand_fps;
}

RealSenseMultiCam::~RealSenseMultiCam()
{
    stop();
}

const char *RealSenseMultiCam::slot_name(CameraSlot slot)
{
    return slot_key(slot).c_str();
}

CameraSlot RealSenseMultiCam::slot_from_choice(int choice)
{
    switch (choice)
    {
    case 1:
        return CameraSlot::Head;
    case 2:
        return CameraSlot::RightHand;
    case 3:
        return CameraSlot::LeftHand;
    default:
        throw std::invalid_argument("invalid camera choice");
    }
}

bool RealSenseMultiCam::init(std::string &err)
{
    err.clear();
    stop();
    log_connected_realsense();

    YAML::Node root;
    try
    {
        root = YAML::LoadFile(impl_->yaml_path);
    }
    catch (const std::exception &e)
    {
        err = std::string("读取相机配置失败: ") + e.what();
        return false;
    }

    const YAML::Node realsense = root["realsense"];
    if (!realsense || !realsense.IsMap())
    {
        err = "配置缺少 realsense 节点: " + impl_->yaml_path;
        return false;
    }

    for (const char *key : kSlotKeys)
    {
        const YAML::Node node = realsense[key];
        if (!node || !node.IsMap())
        {
            err = std::string("配置缺少相机位置: ") + key;
            return false;
        }

        SlotDevice dev;
        dev.slot_id = key;
        dev.label = node["label"] ? node["label"].as<std::string>() : key;
        dev.serial = node["serial"] ? node["serial"].as<std::string>() : "";
        if (dev.serial.empty())
        {
            err = std::string("序列号为空: ") + key;
            return false;
        }
        impl_->slots.emplace(key, std::move(dev));
    }

    // 按固定顺序逐台启动并错开，避免 map 字母序一次挤满 USB 导致末台首帧超时
    bool first = true;
    for (const char *key : kSlotKeys)
    {
        if (std::strcmp(key, "right_hand") == 0 && right_arm_motors_locked())
        {
            std::cout << "[realsense] 右臂锁定，跳过右手相机（配置仍保留，恢复右臂后会再开）\n";
            continue;
        }
        auto &dev = impl_->slots.at(key);
        if (!first)
            std::this_thread::sleep_for(std::chrono::milliseconds(kStaggerBetweenCamsMs));
        first = false;

        const int fps = (dev.slot_id == "head") ? impl_->head_fps : impl_->hand_fps;
        std::string cam_err;
        if (!start_slot_device(dev, impl_->width, impl_->height, fps, cam_err))
        {
            std::cerr << "[realsense] 配置 SN=" << dev.serial
                      << " 不在已连接列表中，请改 config/realsense_cameras.yaml\n";
            log_connected_realsense();
            err = std::string("打开相机失败 [") + dev.slot_id + "] SN=" + dev.serial + ": " + cam_err;
            stop();
            return false;
        }
        std::cout << "RealSense 已启动: " << dev.label << " (" << dev.slot_id
                  << ") SN=" << dev.serial << " " << impl_->width << "x" << impl_->height
                  << "@" << fps << "fps\n";
    }

    return true;
}

namespace
{

CameraFrameData grab_frameset_from_pipeline(
    SlotDevice &dev,
    const rs2::frameset &frames,
    CameraFrameData &out)
{
    const rs2::frameset aligned = dev.align.process(frames);
    const rs2::frame color_f = aligned.get_color_frame();
    const rs2::frame depth_f = aligned.get_depth_frame();
    if (!color_f || !depth_f)
    {
        out.message = "取帧失败: 彩色或深度为空";
        return out;
    }

    const int h = color_f.as<rs2::video_frame>().get_height();
    const int w = color_f.as<rs2::video_frame>().get_width();
    const auto *color_ptr = reinterpret_cast<const uint8_t *>(color_f.get_data());
    const cv::Mat bgr(h, w, CV_8UC3, const_cast<uint8_t *>(color_ptr));
    const cv::Mat depth_m = depth_z16_to_meters(depth_f, dev.depth_scale);

    out.width = w;
    out.height = h;
    mat_bgr_to_vector(bgr.clone(), out.color_bgr);
    mat_depth_to_vector(depth_m, out.depth_m);
    out.K = dev.K;
    out.dist = dev.dist;
    out.ok = true;
    out.message = "ok";
    return out;
}

} // namespace

void RealSenseMultiCam::flush(CameraSlot slot)
{
    const std::string key = slot_key(slot);
    auto it = impl_->slots.find(key);
    if (it == impl_->slots.end() || !it->second.started)
        return;
    flush_pipeline(it->second.pipeline);
}

CameraFrameData RealSenseMultiCam::grab(CameraSlot slot)
{
    CameraFrameData out;
    const std::string key = slot_key(slot);
    auto it = impl_->slots.find(key);
    if (it == impl_->slots.end() || !it->second.started)
    {
        out.message = std::string("相机未初始化: ") + key;
        return out;
    }

    auto &dev = it->second;
    try
    {
        const rs2::frameset frames = grab_newest_frameset(dev.pipeline);
        grab_frameset_from_pipeline(dev, frames, out);
    }
    catch (const std::exception &e)
    {
        out.message = std::string("grab 异常: ") + e.what();
    }
    return out;
}

CameraFrameData RealSenseMultiCam::grab_wait(CameraSlot slot)
{
    CameraFrameData out;
    const std::string key = slot_key(slot);
    auto it = impl_->slots.find(key);
    if (it == impl_->slots.end() || !it->second.started)
    {
        out.message = std::string("相机未初始化: ") + key;
        return out;
    }

    auto &dev = it->second;
    try
    {
        const rs2::frameset frames =
            dev.pipeline.wait_for_frames(static_cast<unsigned int>(kGrabWaitTimeoutMs));
        grab_frameset_from_pipeline(dev, frames, out);
    }
    catch (const std::exception &e)
    {
        out.message = std::string("grab_wait 异常: ") + e.what();
    }
    return out;
}

CameraFrameData RealSenseMultiCam::grab_fresh(CameraSlot slot)
{
    CameraFrameData out;
    const std::string key = slot_key(slot);
    auto it = impl_->slots.find(key);
    if (it == impl_->slots.end() || !it->second.started)
    {
        out.message = std::string("相机未初始化: ") + key;
        return out;
    }

    auto &dev = it->second;
    try
    {
        const rs2::frameset frames = grab_fresh_frameset(dev.pipeline);
        grab_frameset_from_pipeline(dev, frames, out);
    }
    catch (const std::exception &e)
    {
        out.message = std::string("grab_fresh 异常: ") + e.what();
    }
    return out;
}

CameraFrameData RealSenseMultiCam::prepare_frame_for_slot(CameraFrameData frame, CameraSlot slot)
{
    if (!frame.ok || slot != CameraSlot::RightHand)
        return frame;

    const int w = frame.width;
    const int h = frame.height;
    if (w <= 0 || h <= 0)
        return frame;

    cv::Mat bgr(h, w, CV_8UC3, frame.color_bgr.data());
    cv::Mat depth(h, w, CV_32FC1, frame.depth_m.data());

    cv::Mat bgr_rot;
    cv::Mat depth_rot;
    cv::rotate(bgr, bgr_rot, cv::ROTATE_180);
    cv::rotate(depth, depth_rot, cv::ROTATE_180);

    mat_bgr_to_vector(bgr_rot, frame.color_bgr);
    mat_depth_to_vector(depth_rot, frame.depth_m);

    // 图像旋转 180° 后主点与切向畸变需同步变换
    frame.K[2] = static_cast<double>(w - 1) - frame.K[2];
    frame.K[5] = static_cast<double>(h - 1) - frame.K[5];
    frame.dist[2] = -frame.dist[2]; // p1
    frame.dist[3] = -frame.dist[3]; // p2

    return frame;
}

namespace
{

std::string grasp_snap_home_dir()
{
    const char *home = std::getenv("HOME");
    if (home == nullptr || home[0] == '\0')
    {
        if (const passwd *pw = getpwuid(getuid()))
            home = pw->pw_dir;
        else
            home = ".";
    }
    return std::string(home) + "/左右手夹取";
}

void save_one_hand_grasp_snapshot(
    RealSenseMultiCam &cameras,
    CameraSlot slot,
    const char *subdir,
    const char *name_suffix = "")
{
    const std::filesystem::path dir = std::filesystem::path(grasp_snap_home_dir()) / subdir;
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec)
    {
        std::cerr << "[snap] 创建目录失败 " << dir.string() << ": " << ec.message() << "\n";
        return;
    }

    CameraFrameData frame = cameras.grab_fresh(slot);
    frame = RealSenseMultiCam::prepare_frame_for_slot(std::move(frame), slot);
    if (!frame.ok || frame.color_bgr.empty() || frame.width <= 0 || frame.height <= 0)
    {
        std::cerr << "[snap] " << subdir << " 取图失败: " << frame.message << "\n";
        return;
    }

    const auto now = std::chrono::system_clock::now();
    const std::time_t sec = std::chrono::system_clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch()) %
                    1000;
    std::tm tm_local{};
    localtime_r(&sec, &tm_local);

    std::ostringstream fname;
    fname << std::put_time(&tm_local, "%Y%m%d_%H%M%S") << '_'
          << std::setw(3) << std::setfill('0') << ms.count() << name_suffix << ".jpg";

    const std::filesystem::path path = dir / fname.str();
    const cv::Mat bgr(frame.height, frame.width, CV_8UC3, frame.color_bgr.data());
    if (!cv::imwrite(path.string(), bgr))
    {
        std::cerr << "[snap] 保存失败 " << path.string() << "\n";
        return;
    }
    std::cout << "[snap] " << subdir << " 已保存 " << path.string() << "\n";
}

} // namespace

void RealSenseMultiCam::save_grasp_hand_camera_snapshots(bool save_right, bool save_left)
{
    if (save_right)
        save_one_hand_grasp_snapshot(*this, CameraSlot::RightHand, "右手");
    if (save_left)
        save_one_hand_grasp_snapshot(*this, CameraSlot::LeftHand, "左手");
}

void RealSenseMultiCam::stop()
{
    for (auto &kv : impl_->slots)
    {
        if (kv.second.started)
        {
            try
            {
                kv.second.pipeline.stop();
            }
            catch (...)
            {
            }
            kv.second.started = false;
        }
    }
    impl_->slots.clear();
}
