#pragma once

/**
 * M4 Pose：手眼变基座、孔网、fuse、占用关联（薄封装）。
 * 孔枚举归属本模块（拍板）；实现仍调 Bridge.run_tray / run_belt。
 */
#include <cstdint>
#include <string>
#include <vector>

#include "seg_pose_bridge.h"
#include "vision/types.h"

namespace vision
{

struct PoseSolveTrayOpts
{
    bool show_visualization = false;
    CameraSlot vis_slot = CameraSlot::Head;
    bool tray2_markers = false;
};

struct PoseSolveBeltOpts
{
    std::string prefer_name;
    bool show_visualization = false;
    CameraSlot vis_slot = CameraSlot::Head;
    const double *grasp_offset_left = nullptr;
    const double *grasp_offset_right = nullptr;
};

class IPoseSolver
{
public:
    virtual ~IPoseSolver() = default;
    virtual TrayDetectResult solve_tray(
        const std::vector<Frame> &frames,
        const PoseDetectionRecords &yolo,
        const PoseSolveTrayOpts &opts = {},
        std::uint64_t marker_token = 0) = 0;
    /** 只检码并缓存。返回 0 表示未缓存，solve_tray 会自己检。 */
    virtual std::uint64_t prefetch_tray_markers(
        const std::vector<Frame> &frames,
        bool tray2_markers,
        std::string &err)
    {
        (void)frames;
        (void)tray2_markers;
        err.clear();
        return 0;
    }
    virtual void release_tray_markers(std::uint64_t token)
    {
        (void)token;
    }
    virtual BeltDetectResult solve_belt(
        const Frame &frame,
        const PoseSolveBeltOpts &opts = {}) = 0;
};

class BridgePoseSolver final : public IPoseSolver
{
public:
    explicit BridgePoseSolver(SegPoseBridge &bridge);

    TrayDetectResult solve_tray(
        const std::vector<Frame> &frames,
        const PoseDetectionRecords &yolo,
        const PoseSolveTrayOpts &opts = {},
        std::uint64_t marker_token = 0) override;
    std::uint64_t prefetch_tray_markers(
        const std::vector<Frame> &frames,
        bool tray2_markers,
        std::string &err) override;
    void release_tray_markers(std::uint64_t token) override;
    BeltDetectResult solve_belt(
        const Frame &frame,
        const PoseSolveBeltOpts &opts = {}) override;

    SegPoseBridge &bridge() { return bridge_; }

private:
    SegPoseBridge &bridge_;
};

} // namespace vision
