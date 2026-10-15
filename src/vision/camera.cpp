#include "vision/camera.h"

#include "move_box_config.h"
#include "seg_pose_bridge.h"

#include <algorithm>
#include <filesystem>

namespace vision
{

CameraRigConfig CameraRigConfig::make_default(const std::string &project_root)
{
    CameraRigConfig cfg;
    const std::filesystem::path root(project_root.empty() ? project_root_dir() : project_root);
    cfg.sn_yaml_path = (root / "config" / "realsense_cameras.yaml").string();
    cfg.profile_head = "d435i";
    cfg.profile_hand = "d405";
    return cfg;
}

RealSenseCameraBackend::RealSenseCameraBackend(RealSenseMultiCam &cams) : cams_(cams) {}

bool RealSenseCameraBackend::init(std::string &err)
{
    return cams_.init(err);
}

void RealSenseCameraBackend::flush(CameraSlot slot)
{
    cams_.flush(slot);
}

Frame RealSenseCameraBackend::grab(CameraSlot slot)
{
    return cams_.grab(slot);
}

Frame RealSenseCameraBackend::grab_wait(CameraSlot slot)
{
    return cams_.grab_wait(slot);
}

Frame RealSenseCameraBackend::grab_fresh(CameraSlot slot)
{
    return cams_.grab_fresh(slot);
}

void RealSenseCameraBackend::stop()
{
    cams_.stop();
}

std::vector<Frame> grab_head_fused(ICameraBackend &cam, int fuse_n)
{
    const int n = (fuse_n > 0) ? fuse_n : std::max(1, g_move_cfg.tray.fuse_frames);
    cam.flush(CameraSlot::Head);
    std::vector<Frame> frames;
    frames.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i)
    {
        Frame one = cam.grab_wait(CameraSlot::Head);
        one = RealSenseMultiCam::prepare_frame_for_slot(std::move(one), CameraSlot::Head);
        if (!one.ok)
            continue;
        frames.push_back(std::move(one));
    }
    return frames;
}

} // namespace vision
