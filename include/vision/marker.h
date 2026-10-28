#pragma once

/**
 * M3 Marker：只检码 → 盘/带相关几何（薄封装）。
 * 孔枚举 / YOLO 关联仍在 M4 Pose（当前 Python 仍在同次调用里完成，C++ 侧按归属命名切开）。
 */
#include <string>

#include "seg_pose_bridge.h"
#include "vision/types.h"

namespace vision
{

struct MarkerBoardId
{
    static constexpr const char *kTray = "tray";
    static constexpr const char *kTray2 = "tray2";
    static constexpr const char *kBelt = "belt";
};

class IMarkerEngine
{
public:
    virtual ~IMarkerEngine() = default;
    virtual bool ready_tray() const = 0;
    virtual bool ready_belt() const = 0;
    virtual bool ready_aruco() const = 0;
    virtual ArucoDetectResult detect_aruco(
        const Frame &frame,
        bool show_visualization = false,
        CameraSlot vis_slot = CameraSlot::Head) = 0;
};

/** 经 SegPoseBridge 的薄门面；板级 detect 由 PoseSolver 编排（因 fuse/孔网在 Pose）。 */
class BridgeMarkerEngine final : public IMarkerEngine
{
public:
    explicit BridgeMarkerEngine(SegPoseBridge &bridge);

    bool ready_tray() const override;
    bool ready_belt() const override;
    bool ready_aruco() const override;
    ArucoDetectResult detect_aruco(
        const Frame &frame,
        bool show_visualization = false,
        CameraSlot vis_slot = CameraSlot::Head) override;

    SegPoseBridge &bridge() { return bridge_; }

private:
    SegPoseBridge &bridge_;
};

} // namespace vision
