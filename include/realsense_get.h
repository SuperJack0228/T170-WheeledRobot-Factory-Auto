#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

/** 相机部位，与 config/realsense_cameras.yaml 键名对应 */
enum class CameraSlot
{
    Head,
    RightHand,
    LeftHand,
};

/** 单帧：1280×720 BGR + 对齐深度 (m) + 内参/畸变 */
struct CameraFrameData
{
    int width = 0;
    int height = 0;
    std::vector<uint8_t> color_bgr; /**< BGR 行连续, size = H*W*3 */
    std::vector<float> depth_m;     /**< 与彩色同尺寸, size = H*W；无效为 nan */
    std::array<double, 9> K{};      /**< 3×3 行主序 */
    std::array<double, 5> dist{};   /**< k1,k2,p1,p2,k3 */
    bool ok = false;
    std::string message;
};

/** 从 YAML 初始化三台 RealSense，按部位取对齐彩色/深度帧 */
class RealSenseMultiCam
{
public:
    RealSenseMultiCam(
        std::string yaml_path,
        int width = 1280,
        int height = 720,
        int head_fps = 15,
        int hand_fps = 15);
    ~RealSenseMultiCam();

    RealSenseMultiCam(const RealSenseMultiCam &) = delete;
    RealSenseMultiCam &operator=(const RealSenseMultiCam &) = delete;

    bool init(std::string &err);
    /** 排空该相机 pipeline 内全部缓冲帧（不返回图像） */
    void flush(CameraSlot slot);
    /** 取该部位最新一帧（先排空 pipeline 缓冲，避免手臂移动期间积压的旧图） */
    CameraFrameData grab(CameraSlot slot);
    /** 阻塞等待下一帧（不排空）。多帧融合时先 flush 再连续 grab_wait。 */
    CameraFrameData grab_wait(CameraSlot slot);
    /** 手相机识别用：彻底 flush 后再 wait 两帧，尽量保证停稳后的新图 */
    CameraFrameData grab_fresh(CameraSlot slot);
    void stop();

    static const char *slot_name(CameraSlot slot);
    static CameraSlot slot_from_choice(int choice);

    /** 右手相机：RGB/深度旋转 180° 并修正内参；头/左手原样返回 */
    static CameraFrameData prepare_frame_for_slot(CameraFrameData frame, CameraSlot slot);

    /** 夹取抬起后保存手相机图：~/左右手夹取/右手|左手/时间戳.jpg（各 1 次） */
    void save_grasp_hand_camera_snapshots(bool save_right, bool save_left);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
