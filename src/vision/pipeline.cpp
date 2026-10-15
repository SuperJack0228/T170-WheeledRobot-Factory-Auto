#include "vision/pipeline.h"

#include "Ti5_socketcan.h"
#include "seg_pose_bridge.h"

namespace vision
{

HeadTrayPerception perceive_tray_from_frames(
    IDlEngine &dl,
    IPoseSolver &pose,
    std::vector<Frame> frames,
    const PerceiveTrayOptions &opts)
{
    HeadTrayPerception out;
    out.frames = std::move(frames);
    if (out.frames.empty())
    {
        out.tray.message = "取帧失败";
        return out;
    }
    if (app_stop_requested() || hardware_abort_requested())
    {
        out.tray.message = "已中止";
        return out;
    }

    const Frame &frame = out.frames.back();
    if (opts.run_yolo)
    {
        DlRunOpts dl_opts;
        dl_opts.algorithm_id = algorithm_id_for_slot(CameraSlot::Head);
        dl_opts.show_visualization = opts.show_yolo_visualization;
        dl_opts.vis_slot = CameraSlot::Head;
        dl_opts.engine_id = opts.engine_id;
        const DlResult dl_res = dl.run(frame, "tray_parts", dl_opts);
        out.yolo = dl_res.raw;
        out.yolo_records =
            pose_records_from_run(out.yolo, CameraSlot::Head, frame, -1, false);
    }

    PoseSolveTrayOpts pose_opts;
    pose_opts.show_visualization = opts.show_tray_visualization;
    pose_opts.vis_slot = CameraSlot::Head;
    pose_opts.tray2_markers = opts.tray2_markers;
    out.tray = pose.solve_tray(out.frames, out.yolo_records, pose_opts);
    return out;
}

HeadTrayPerception perceive_tray(
    ICameraBackend &camera,
    IDlEngine &dl,
    IPoseSolver &pose,
    const PerceiveTrayOptions &opts)
{
    return perceive_tray_from_frames(
        dl, pose, grab_head_fused(camera, opts.fuse_n), opts);
}

HeadBeltPerception perceive_belt(
    ICameraBackend &camera,
    IPoseSolver &pose,
    const PoseSolveBeltOpts &opts)
{
    HeadBeltPerception out;
    if (app_stop_requested() || hardware_abort_requested())
    {
        out.belt.message = "已中止";
        return out;
    }
    camera.flush(CameraSlot::Head);
    out.frame = camera.grab_wait(CameraSlot::Head);
    out.frame =
        RealSenseMultiCam::prepare_frame_for_slot(std::move(out.frame), CameraSlot::Head);
    if (!out.frame.ok)
    {
        out.belt.message = "取帧失败";
        return out;
    }
    out.belt = pose.solve_belt(out.frame, opts);
    return out;
}

HeadTrayPerception perceive_tray_head(
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    const PerceiveTrayOptions &opts)
{
    RealSenseCameraBackend cam(cameras);
    BridgeDlEngine dl(bridge);
    BridgePoseSolver pose(bridge);
    return perceive_tray(cam, dl, pose, opts);
}

HeadBeltPerception perceive_belt_head(
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    const std::string &prefer_name,
    bool show_visualization,
    const double *grasp_offset_left,
    const double *grasp_offset_right)
{
    RealSenseCameraBackend cam(cameras);
    BridgePoseSolver pose(bridge);
    PoseSolveBeltOpts opts;
    opts.prefer_name = prefer_name;
    opts.show_visualization = show_visualization;
    opts.vis_slot = CameraSlot::Head;
    opts.grasp_offset_left = grasp_offset_left;
    opts.grasp_offset_right = grasp_offset_right;
    if (!bridge.belt_engine_ready())
    {
        HeadBeltPerception out;
        cam.flush(CameraSlot::Head);
        out.frame = cam.grab_wait(CameraSlot::Head);
        out.frame =
            RealSenseMultiCam::prepare_frame_for_slot(std::move(out.frame), CameraSlot::Head);
        out.belt.message = "传送带引擎未就绪";
        return out;
    }
    return perceive_belt(cam, pose, opts);
}

} // namespace vision
