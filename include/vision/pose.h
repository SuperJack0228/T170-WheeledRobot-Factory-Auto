#pragma once

/**
 * M4 Pose：手眼变基座、孔网、fuse、占用关联（薄封装）。
 * 孔枚举归属本模块（拍板）；实现仍调 Bridge.run_tray / run_belt。
 */
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
        const PoseSolveTrayOpts &opts = {}) = 0;
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
        const PoseSolveTrayOpts &opts = {}) override;
    BeltDetectResult solve_belt(
        const Frame &frame,
        const PoseSolveBeltOpts &opts = {}) override;

    SegPoseBridge &bridge() { return bridge_; }

private:
    SegPoseBridge &bridge_;
};

} // namespace vision
