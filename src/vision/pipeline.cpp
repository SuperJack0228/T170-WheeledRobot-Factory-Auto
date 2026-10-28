#include "vision/pipeline.h"

#include "Ti5_socketcan.h"
#include "seg_pose_bridge.h"

#include <chrono>
#include <iostream>
#include <thread>

namespace vision
{

namespace
{

struct MarkerTokenGuard
{
    IPoseSolver *pose = nullptr;
    std::uint64_t token = 0;
    ~MarkerTokenGuard()
    {
        if (pose != nullptr && token != 0)
            pose->release_tray_markers(token);
    }
};

} // namespace

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
    MarkerTokenGuard guard;
    guard.pose = &pose;
    if (opts.run_yolo)
    {
        DlResult dl_res;
        std::string yolo_err;
        std::string mark_err;
        const auto t0 = std::chrono::steady_clock::now();
        std::thread yolo_th([&] {
            try
            {
                DlRunOpts dl_opts;
                dl_opts.algorithm_id = algorithm_id_for_slot(CameraSlot::Head);
                dl_opts.show_visualization = opts.show_yolo_visualization;
                dl_opts.vis_slot = CameraSlot::Head;
                dl_opts.engine_id = opts.engine_id;
                dl_res = dl.run(frame, "tray_parts", dl_opts);
            }
            catch (const std::exception &e)
            {
                yolo_err = e.what();
            }
        });
        std::thread mark_th([&] {
            try
            {
                guard.token = pose.prefetch_tray_markers(out.frames, opts.tray2_markers, mark_err);
            }
            catch (const std::exception &e)
            {
                mark_err = e.what();
            }
        });
        yolo_th.join();
        mark_th.join();
        const auto wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::steady_clock::now() - t0)
                                 .count();
        std::cout << "[vision] 料盘 YOLO∥二维码 " << wall_ms << " ms"
                  << (guard.token != 0 ? "" : "（码预检未缓存，解算时重检）") << "\n";
        if (!mark_err.empty() && guard.token == 0)
            std::cerr << "[vision] 料盘并行检码: " << mark_err << "\n";
        if (!yolo_err.empty())
        {
            out.yolo.ok = false;
            out.yolo.message = yolo_err;
        }
        else
        {
            out.yolo = std::move(dl_res.raw);
            out.yolo_records =
                pose_records_from_run(out.yolo, CameraSlot::Head, frame, -1, false);
        }
    }

    PoseSolveTrayOpts pose_opts;
    pose_opts.show_visualization = opts.show_tray_visualization;
    pose_opts.vis_slot = CameraSlot::Head;
    pose_opts.tray2_markers = opts.tray2_markers;
    const std::uint64_t token = guard.token;
    guard.token = 0;
    out.tray = pose.solve_tray(out.frames, out.yolo_records, pose_opts, token);
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
    const PoseSolveBeltOpts &opts,
    IDlEngine *dl)
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
    if (dl == nullptr)
    {
        out.belt = pose.solve_belt(out.frame, opts);
        return out;
    }

    DlResult dl_res;
    std::string yolo_err;
    const auto t0 = std::chrono::steady_clock::now();
    std::thread yolo_th([&] {
        try
        {
            DlRunOpts dl_opts;
            dl_opts.algorithm_id = 2;
            dl_opts.show_visualization = false;
            dl_res = dl->run(out.frame, "belt_presence", dl_opts);
        }
        catch (const std::exception &e)
        {
            yolo_err = e.what();
        }
    });
    out.belt = pose.solve_belt(out.frame, opts);
    yolo_th.join();
    const auto wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - t0)
                             .count();
    std::cout << "[vision] 传送带 YOLO∥二维码 " << wall_ms << " ms\n";
    out.yolo_ran = true;
    if (!yolo_err.empty())
    {
        out.yolo.ok = false;
        out.yolo.message = yolo_err;
    }
    else
        out.yolo = std::move(dl_res.raw);
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
    const double *grasp_offset_right,
    bool parallel_yolo)
{
    RealSenseCameraBackend cam(cameras);
    BridgePoseSolver pose(bridge);
    BridgeDlEngine dl(bridge);
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
    return perceive_belt(cam, pose, opts, parallel_yolo ? &dl : nullptr);
}

} // namespace vision
