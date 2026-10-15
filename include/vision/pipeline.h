#pragma once

/**
 * M5 编排：Camera → DL + Marker/Pose；对外仍用 HeadTrayPerception / HeadBeltPerception。
 * SegPoseBridge 保留类名，本文件逐渐成为唯一胶水入口。
 */
#include "vision/camera.h"
#include "vision/dl.h"
#include "vision/marker.h"
#include "vision/pose.h"

namespace vision
{

/** 料盘：取帧 → DL(tray_parts) → Pose.solve_tray（内含码+孔+关联）。 */
HeadTrayPerception perceive_tray(
    ICameraBackend &camera,
    IDlEngine &dl,
    IPoseSolver &pose,
    const PerceiveTrayOptions &opts = {});

/** 已有多帧时跳过 Camera。 */
HeadTrayPerception perceive_tray_from_frames(
    IDlEngine &dl,
    IPoseSolver &pose,
    std::vector<Frame> frames,
    const PerceiveTrayOptions &opts = {});

/** 传送带：取帧 → Pose.solve_belt（码定位）。DL 门禁由 Skill 另调 belt_presence。 */
HeadBeltPerception perceive_belt(
    ICameraBackend &camera,
    IPoseSolver &pose,
    const PoseSolveBeltOpts &opts);

/**
 * 便利：用 RealSenseMultiCam + SegPoseBridge 组装四模块再感知。
 * 与历史 perceive_tray_head / perceive_belt_head 行为一致。
 */
HeadTrayPerception perceive_tray_head(
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    const PerceiveTrayOptions &opts = {});

HeadBeltPerception perceive_belt_head(
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    const std::string &prefer_name,
    bool show_visualization = false,
    const double *grasp_offset_left = nullptr,
    const double *grasp_offset_right = nullptr);

} // namespace vision
