#include "vision/dl.h"

#include "vision_debug.h"

#include <algorithm>
#include <cstring>
#include <iostream>

#include <opencv2/imgcodecs.hpp>

namespace vision
{

void ModelRegistry::clear()
{
    entries_.clear();
}

void ModelRegistry::upsert(ModelEntry entry)
{
    const std::string key = entry.key;
    entries_[key] = std::move(entry);
}

void ModelRegistry::register_defaults()
{
    upsert(ModelEntry{
        "tray_parts",
        "detect",
        "models/best1015.pt",
        2,
        "料盘关联用；业务仍按 class 挂孔（S2 延期）",
    });
    upsert(ModelEntry{
        "belt_presence",
        "detect",
        "models/best1015.pt",
        2,
        "传送带 ROI 有无+数量；不参与定位",
    });
}

bool ModelRegistry::has(const std::string &key) const
{
    return entries_.find(key) != entries_.end();
}

const ModelEntry *ModelRegistry::get(const std::string &key) const
{
    const auto it = entries_.find(key);
    return it == entries_.end() ? nullptr : &it->second;
}

DlResult dl_from_pose_run(const PoseRunResult &raw, const std::string &model_key)
{
    DlResult out;
    out.model_key = model_key;
    out.raw = raw;
    out.ok = raw.ok;
    out.message = raw.message;
    int n = 0;
    for (const auto &t : raw.targets)
    {
        DlDetection d;
        d.class_id = t.class_id;
        d.class_name = t.class_name;
        d.confidence = t.confidence;
        d.success = t.success;
        d.u = t.box_u;
        d.v = t.box_v;
        if (t.success)
            ++n;
        out.detections.push_back(std::move(d));
    }
    out.count = n;
    out.has_part = n > 0;
    return out;
}

BridgeDlEngine::BridgeDlEngine(SegPoseBridge &bridge) : bridge_(bridge)
{
    registry_.register_defaults();
}

bool BridgeDlEngine::load(const ModelRegistry &reg, std::string &err)
{
    registry_ = reg;
    if (!registry_.has("tray_parts") && !registry_.has("belt_presence"))
    {
        err = "ModelRegistry 为空：至少需要 tray_parts 或 belt_presence";
        return false;
    }
    err.clear();
    return true;
}

DlResult BridgeDlEngine::run(
    const Frame &frame,
    const std::string &model_key,
    const DlRunOpts &opts)
{
    if (opts.use_roi)
        return run_roi_count(frame, model_key, opts);
    return run_full_frame(frame, model_key, opts);
}

DlResult BridgeDlEngine::run_full_frame(
    const Frame &frame,
    const std::string &model_key,
    const DlRunOpts &opts)
{
    int algo = opts.algorithm_id;
    if (algo < 0)
    {
        if (const ModelEntry *e = registry_.get(model_key))
            algo = e->default_algorithm;
    }
    const PoseRunResult raw = bridge_.run_yolo(
        frame, algo, opts.show_visualization, opts.vis_slot, -1, opts.engine_id);
    return dl_from_pose_run(raw, model_key);
}

DlResult BridgeDlEngine::run_roi_count(
    const Frame &frame,
    const std::string &model_key,
    const DlRunOpts &opts)
{
    DlResult out;
    out.model_key = model_key.empty() ? "belt_presence" : model_key;
    if (!frame.ok || frame.color_bgr.empty() || frame.width <= 0 || frame.height <= 0)
    {
        out.message = "无效帧";
        return out;
    }
    if (opts.roi_w <= 0 || opts.roi_h <= 0)
    {
        out.message = "yolo_roi 宽高未配置";
        return out;
    }

    const int x0 = std::max(0, opts.roi_x);
    const int y0 = std::max(0, opts.roi_y);
    const int x1 = std::min(frame.width, opts.roi_x + opts.roi_w);
    const int y1 = std::min(frame.height, opts.roi_y + opts.roi_h);
    const int rw = x1 - x0;
    const int rh = y1 - y0;
    if (rw < 8 || rh < 8)
    {
        out.message = "yolo_roi 裁切后太小或越界";
        return out;
    }

    Frame crop;
    crop.width = rw;
    crop.height = rh;
    crop.K = frame.K;
    crop.K[2] -= static_cast<double>(x0);
    crop.K[5] -= static_cast<double>(y0);
    crop.dist = frame.dist;
    crop.color_bgr.resize(static_cast<size_t>(rw) * static_cast<size_t>(rh) * 3u);
    for (int y = 0; y < rh; ++y)
    {
        const uint8_t *src =
            frame.color_bgr.data() +
            (static_cast<size_t>(y0 + y) * static_cast<size_t>(frame.width) +
             static_cast<size_t>(x0)) *
                3u;
        uint8_t *dst =
            crop.color_bgr.data() + static_cast<size_t>(y) * static_cast<size_t>(rw) * 3u;
        std::memcpy(dst, src, static_cast<size_t>(rw) * 3u);
    }
    if (frame.depth_m.size() ==
        static_cast<size_t>(frame.width) * static_cast<size_t>(frame.height))
    {
        crop.depth_m.resize(static_cast<size_t>(rw) * static_cast<size_t>(rh));
        for (int y = 0; y < rh; ++y)
        {
            const float *src =
                frame.depth_m.data() +
                (static_cast<size_t>(y0 + y) * static_cast<size_t>(frame.width) +
                 static_cast<size_t>(x0));
            float *dst = crop.depth_m.data() + static_cast<size_t>(y) * static_cast<size_t>(rw);
            std::memcpy(dst, src, static_cast<size_t>(rw) * sizeof(float));
        }
    }
    crop.ok = true;

    if (vision_debug_annotate())
    {
        try
        {
            cv::Mat bgr(crop.height, crop.width, CV_8UC3, crop.color_bgr.data());
            const std::string path = vision_debug_make_path("head", "belt_yolo_input");
            if (!path.empty() && cv::imwrite(path, bgr))
                std::cout << "[dl] YOLO输入原图 " << path << " " << crop.width << "x" << crop.height
                          << "\n";
        }
        catch (const std::exception &e)
        {
            std::cerr << "[dl] 保存 YOLO 输入图失败: " << e.what() << "\n";
        }
    }

    int algo = opts.algorithm_id >= 0 ? opts.algorithm_id : 2;
    const PoseRunResult yolo =
        bridge_.run_yolo(crop, algo, false, CameraSlot::Head, -1, opts.engine_id);
    out.raw = yolo;
    if (!yolo.ok)
    {
        out.message = "YOLO失败: " + yolo.message;
        return out;
    }

    std::vector<int> part_ids = opts.part_class_ids;
    if (part_ids.empty())
        part_ids = {0, 1, 2};

    auto is_part = [&](int cid) {
        for (int id : part_ids)
        {
            if (id == cid)
                return true;
        }
        return false;
    };

    int n = 0;
    int n_fail = 0;
    int n_conf = 0;
    int n_class = 0;
    for (const auto &t : yolo.targets)
    {
        DlDetection d;
        d.class_id = t.class_id;
        d.class_name = t.class_name;
        d.confidence = t.confidence;
        d.success = t.success;
        d.u = t.box_u;
        d.v = t.box_v;
        out.detections.push_back(d);
        std::cout << "[dl] 模型框 class=" << t.class_id << " " << t.class_name
                  << " conf=" << t.confidence << " success=" << (t.success ? 1 : 0)
                  << " uv=(" << t.box_u << "," << t.box_v << ") " << t.message << "\n";
        if (!t.success)
        {
            ++n_fail;
            continue;
        }
        if (static_cast<double>(t.confidence) < opts.min_conf)
        {
            ++n_conf;
            continue;
        }
        if (!is_part(t.class_id))
        {
            ++n_class;
            continue;
        }
        ++n;
    }
    out.ok = true;
    out.count = n;
    out.has_part = n > 0;
    out.message = "ok";
    std::cout << "[dl] 模型原始框=" << yolo.targets.size() << " 通过=" << n
              << " 丢弃 success失败=" << n_fail << " 低置信=" << n_conf << " 非零件类=" << n_class
              << "\n";
    return out;
}

} // namespace vision
