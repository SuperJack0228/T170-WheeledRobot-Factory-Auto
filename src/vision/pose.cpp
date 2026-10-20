#include "vision/pose.h"

namespace vision
{

BridgePoseSolver::BridgePoseSolver(SegPoseBridge &bridge) : bridge_(bridge) {}

TrayDetectResult BridgePoseSolver::solve_tray(
    const std::vector<Frame> &frames,
    const PoseDetectionRecords &yolo,
    const PoseSolveTrayOpts &opts,
    std::uint64_t marker_token)
{
    if (!bridge_.tray_engine_ready())
    {
        if (marker_token != 0)
            bridge_.drop_tray_locate(marker_token);
        TrayDetectResult r;
        r.message = "料盘引擎未就绪";
        return r;
    }
    return bridge_.run_tray(
        frames, yolo, opts.show_visualization, opts.vis_slot, opts.tray2_markers, marker_token);
}

std::uint64_t BridgePoseSolver::prefetch_tray_markers(
    const std::vector<Frame> &frames,
    bool tray2_markers,
    std::string &err)
{
    return bridge_.locate_tray_markers(frames, tray2_markers, err);
}

void BridgePoseSolver::release_tray_markers(std::uint64_t token)
{
    bridge_.drop_tray_locate(token);
}

BeltDetectResult BridgePoseSolver::solve_belt(
    const Frame &frame,
    const PoseSolveBeltOpts &opts)
{
    if (!bridge_.belt_engine_ready())
    {
        BeltDetectResult r;
        r.message = "传送带引擎未就绪";
        return r;
    }
    return bridge_.run_belt(
        frame,
        opts.prefer_name,
        opts.show_visualization,
        opts.vis_slot,
        opts.grasp_offset_left,
        opts.grasp_offset_right);
}

} // namespace vision
