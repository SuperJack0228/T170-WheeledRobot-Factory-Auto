#pragma once

/**
 * M1 Camera：统一取帧接口；本机 SN 仍读 config/realsense_cameras.yaml。
 * 型号 profile：config/camera_profile_*.yaml（分辨率/FPS 文档约定；运行时仍可由调用方传入）。
 */
#include <string>
#include <vector>

#include "vision/types.h"

namespace vision
{

struct CameraSlotConfig
{
    CameraSlot slot = CameraSlot::Head;
    std::string serial;
    std::string profile; /**< 如 d435i / d405 */
    int width = 1280;
    int height = 720;
    int fps = 15;
};

/** 本机相机编组：SN 表 + 默认 profile 名（不强制改 RealSenseMultiCam 构造路径）。 */
struct CameraRigConfig
{
    std::string sn_yaml_path;      /**< 默认 realsense_cameras.yaml */
    std::string profile_head;      /**< d435i */
    std::string profile_hand;      /**< d405 */
    int head_width = 1280;
    int head_height = 720;
    int head_fps = 15;
    int hand_width = 1280;
    int hand_height = 720;
    int hand_fps = 15;

    /** 填工程默认路径与 profile 名；不读 yaml 数值（数值仍由 move_box cameras: 传入 ctor）。 */
    static CameraRigConfig make_default(const std::string &project_root);
};

class ICameraBackend
{
public:
    virtual ~ICameraBackend() = default;
    virtual bool init(std::string &err) = 0;
    virtual void flush(CameraSlot slot) = 0;
    virtual Frame grab(CameraSlot slot) = 0;
    virtual Frame grab_wait(CameraSlot slot) = 0;
    virtual Frame grab_fresh(CameraSlot slot) = 0;
    virtual void stop() = 0;
    /** 逃生口：旧代码仍拿 RealSenseMultiCam&；模块内优先走本接口。 */
    virtual RealSenseMultiCam *as_realsense() { return nullptr; }
};

/** 包装已有 RealSenseMultiCam（不拥有），满足 ICameraBackend。 */
class RealSenseCameraBackend final : public ICameraBackend
{
public:
    explicit RealSenseCameraBackend(RealSenseMultiCam &cams);

    bool init(std::string &err) override;
    void flush(CameraSlot slot) override;
    Frame grab(CameraSlot slot) override;
    Frame grab_wait(CameraSlot slot) override;
    Frame grab_fresh(CameraSlot slot) override;
    void stop() override;
    RealSenseMultiCam *as_realsense() override { return &cams_; }
    RealSenseMultiCam &raw() { return cams_; }

private:
    RealSenseMultiCam &cams_;
};

/** 头相机：flush → 连续 grab_wait → prepare。fuse_n<=0 时由调用方决定默认。 */
std::vector<Frame> grab_head_fused(ICameraBackend &cam, int fuse_n);

} // namespace vision
