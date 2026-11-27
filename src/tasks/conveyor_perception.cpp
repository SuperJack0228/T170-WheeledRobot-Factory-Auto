#include "internal/runtime.h"
#include "t170/skills/conveyor.h"

#include "t170/telemetry/factory_status.h"
#include "t170/telemetry/dispatch_metrics.h"
#include "t170/motion/arm.h"
#include "t170/common/units.h"
#include "t170/motion/cartesian_tracking.h"
#include "t170/motion/retreat.h"
#include "t170/motion/servo_handoff.h"
#include "t170/vision/wrist_observer.h"
#include "t170/motion/servo.h"
#include "t170/vision/head_transform.h"
#include "t170/hardware/head.h"
#include "t170/skills/legacy/profiles.h"
#include "t170/skills/legacy/runtime.h"
#include "t170/vision/python_bridge.h"
#include "t170/vision/dl.h"
#include "t170/vision/debug.h"
#include "t170/hardware/device_bus.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <cerrno>
#include <csignal>
#include <mutex>
#include "t170/motion/retreat_gate.h"
#include <sys/select.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using std::vector;
using std::string;
using std::thread;


namespace orch {
void log_belt_used_box(const char *name,const char *role,int index,const BeltPartMark &part);
bool apply_fixed_belt_origin_z(BeltDetectResult &det, const ConveyorStationConfig &c, const char *tag)
{
    const auto &catalog=g_move_cfg.conveyor_heights;
    if (!catalog.enabled && !c.use_fixed_origin_z)return true;
    if (!det.in_robot || !std::isfinite(det.origin_z))return false;
    const double cam_z = det.origin_z;
    double plane_z = c.fixed_plane_z();
    if(catalog.enabled) {
        t170::config::ConveyorHeightMatch match;std::string error;
        if(!t170::config::match_conveyor_height(catalog,det.used_ids,c.height_profile,
            c.fixed_origin_z_m,c.fixed_origin_ref_waist_z_m,c.waist_z,match,error)) {
            det.ok=false;det.message=error;
            if(g_hw){g_hw->last_belt_ok=false;g_hw->last_belt_message=error;}
            std::cerr<<"[conveyor_height] reject task="<<tag<<" reason="<<error<<'\n';return false;
        }
        plane_z=match.plane_z_m;
        std::cout<<std::fixed<<std::setprecision(4)<<"[conveyor_height] task="<<tag
            <<" machine="<<match.machine.machine_id<<" business="<<match.machine.business_id
            <<" info_id="<<match.machine.info_id<<" qr_plane_height_cm="<<match.machine.qr_plane_height_m*100
            <<" reference="<<catalog.reference_machine<<" delta_m="<<match.delta_height_m
            <<" waist_z_m="<<c.waist_z<<" plane_z_m="<<plane_z<<'\n';
    }
    const double dz = plane_z - cam_z;
    det.origin_z = plane_z;
    if (det.have_grasp_l && std::isfinite(det.grasp_lz))
        det.grasp_lz += dz;
    if (det.have_grasp_r && std::isfinite(det.grasp_rz))
        det.grasp_rz += dz;
    std::cout << std::fixed << std::setprecision(4)
              << "[orch] " << tag << " 码平面 Z 不用相机：相机=" << cam_z
              << "m 固定=" << plane_z
              << "m（标定腰 " << c.fixed_origin_ref_waist_z_m
              << "m 时 " << c.fixed_origin_z_m
              << "m，当前腰 " << c.waist_z << "m）ΔZ=" << dz << "m\n";
    return true;
}

const char *belt_yolo_class_zh(int class_id)
{
    switch (class_id)
    {
    case kGraspDetectClassId:
        return "立件";
    case kEmptyHoleClassId:
        return "空孔";
    case kFallenClassId:
        return "倒件";
    default:
        return "未知";
    }
}



bool cam_to_marker(
    const BeltDetectResult &det, double cx, double cy, double cz, double &bx, double &by, double &bz)
{
    if (!det.have_cam_pose)
        return false;
    cv::Mat rvec(3, 1, CV_64F);
    cv::Mat tvec(3, 1, CV_64F);
    for (int i = 0; i < 3; ++i)
    {
        rvec.at<double>(i) = det.rvec[i];
        tvec.at<double>(i) = det.tvec[i];
    }
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat d = (cv::Mat_<double>(3, 1) << cx - tvec.at<double>(0), cy - tvec.at<double>(1),
                  cz - tvec.at<double>(2));
    cv::Mat b = R.t() * d;
    bx = b.at<double>(0);
    by = b.at<double>(1);
    bz = b.at<double>(2);
    return std::isfinite(bx) && std::isfinite(by) && std::isfinite(bz);
}

float belt_box_iou(const BeltPartMark &a, const BeltPartMark &b)
{
    const float x0 = std::max(a.x0, b.x0);
    const float y0 = std::max(a.y0, b.y0);
    const float x1 = std::min(a.x1, b.x1);
    const float y1 = std::min(a.y1, b.y1);
    const float w = x1 - x0;
    const float h = y1 - y0;
    if (!(w > 0.f) || !(h > 0.f))
        return 0.f;
    const float inter = w * h;
    const float aa = std::max(0.f, a.x1 - a.x0) * std::max(0.f, a.y1 - a.y0);
    const float bb = std::max(0.f, b.x1 - b.x0) * std::max(0.f, b.y1 - b.y0);
    const float uni = aa + bb - inter;
    if (!(uni > 0.f))
        return 0.f;
    return inter / uni;
}

std::vector<BeltPartMark> merge_overlapping_belt_parts(const std::vector<BeltPartMark> &raw)
{
    const int n = static_cast<int>(raw.size());
    std::vector<int> parent(static_cast<size_t>(n));
    std::vector<int> rank(static_cast<size_t>(n), 0);
    for (int i = 0; i < n; ++i)
        parent[static_cast<size_t>(i)] = i;
    auto find = [&](int i) {
        while (parent[static_cast<size_t>(i)] != i)
        {
            parent[static_cast<size_t>(i)] = parent[static_cast<size_t>(parent[static_cast<size_t>(i)])];
            i = parent[static_cast<size_t>(i)];
        }
        return i;
    };
    auto unite = [&](int a, int b) {
        a = find(a);
        b = find(b);
        if (a == b)
            return;
        if (rank[static_cast<size_t>(a)] < rank[static_cast<size_t>(b)])
            std::swap(a, b);
        parent[static_cast<size_t>(b)] = a;
        if (rank[static_cast<size_t>(a)] == rank[static_cast<size_t>(b)])
            ++rank[static_cast<size_t>(a)];
    };
    for (int i = 0; i < n; ++i)
    {
        for (int j = i + 1; j < n; ++j)
        {
            if (belt_box_iou(raw[static_cast<size_t>(i)], raw[static_cast<size_t>(j)]) >= kBeltPartMergeIou)
                unite(i, j);
        }
    }
    std::vector<std::vector<int>> groups(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i)
        groups[static_cast<size_t>(find(i))].push_back(i);
    std::vector<BeltPartMark> out;
    for (const auto &group : groups)
    {
        if (group.empty())
            continue;
        float work_conf = -1.f;
        float fall_conf = -1.f;
        int work_i = -1;
        int fall_i = -1;
        int best_i = group.front();
        for (int idx : group)
        {
            const BeltPartMark &m = raw[static_cast<size_t>(idx)];
            if (m.conf > raw[static_cast<size_t>(best_i)].conf)
                best_i = idx;
            if (m.class_id == kGraspDetectClassId && m.conf > work_conf)
            {
                work_conf = m.conf;
                work_i = idx;
            }
            else if (m.class_id == kFallenClassId && m.conf > fall_conf)
            {
                fall_conf = m.conf;
                fall_i = idx;
            }
        }
        int pick = best_i;
        if (fall_i >= 0 && (work_i < 0 || fall_conf > work_conf + kFallenClearlyHigher))
            pick = fall_i;
        else if (work_i >= 0)
            pick = work_i;
        out.push_back(raw[static_cast<size_t>(pick)]);
    }
    return out;
}

BeltPartList detect_belt_parts_full(
    SegPoseBridge &bridge,
    const CameraFrameData &frame,
    const ConveyorStationConfig &c,
    const BeltDetectResult &det,
    const char *tag,
    const PoseRunResult *precomputed)
{
    BeltPartList out;
    const char *name = (tag && tag[0]) ? tag : "belt";
    PoseRunResult yolo;
    if (precomputed != nullptr)
    {
        if (!precomputed->ok)
        {
            out.message = precomputed->message.empty() ? "YOLO失败" : precomputed->message;
            return out;
        }
        yolo = *precomputed;
    }
    else
    {
        vision::DlRunOpts opts;
        opts.algorithm_id = 2;
        vision::BridgeDlEngine dl(bridge);
        const vision::DlResult dl_res = dl.run(frame, "belt_presence", opts);
        if (!dl_res.ok)
        {
            out.message = dl_res.message;
            return out;
        }
        yolo = dl_res.raw;
    }
    std::vector<int> part_ids = c.yolo_part_class_ids;
    if (part_ids.empty())
        part_ids = {kGraspDetectClassId, kFallenClassId};
    std::vector<BeltPartMark> raw_parts;
    cv::Mat vis;
    if (frame.ok && !frame.color_bgr.empty())
    {
        vis = cv::Mat(frame.height, frame.width, CV_8UC3,
                      const_cast<uint8_t *>(frame.color_bgr.data()))
                  .clone();
    }
    for (const auto &t : yolo.targets)
    {
        if (!t.success || static_cast<double>(t.confidence) < c.yolo_min_conf)
            continue;
        bool part = false;
        for (int id : part_ids)
        {
            if (id == t.class_id)
            {
                part = true;
                break;
            }
        }
        if (!part)
            continue;
        BeltPartMark m;
        m.class_id = t.class_id;
        m.conf = t.confidence;
        m.x0 = t.box_x0;
        m.y0 = t.box_y0;
        m.x1 = t.box_x1;
        m.y1 = t.box_y1;
        m.have_marker = cam_to_marker(
            det, t.pose_4x4[3], t.pose_4x4[7], t.pose_4x4[11], m.x, m.y, m.z);
        if (m.have_marker)
            raw_parts.push_back(m);
        else
            log_belt_used_box(name, "夹取", static_cast<int>(raw_parts.size()), m);
        if (!vis.empty() && m.x0 >= 0.f && m.y0 >= 0.f)
        {
            cv::rectangle(
                vis, cv::Point(cvRound(m.x0), cvRound(m.y0)),
                cv::Point(cvRound(m.x1), cvRound(m.y1)), cv::Scalar(0, 255, 0), 2);
            std::ostringstream lab;
            lab << std::fixed << std::setprecision(2) << m.class_id << " " << m.conf;
            if (m.have_marker)
                lab << " (" << m.x << "," << m.y << ")";
            cv::putText(
                vis, lab.str(), cv::Point(cvRound(m.x0), std::max(16, cvRound(m.y0) - 4)),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        }
    }
    if (!vis.empty())
    {
        std::ostringstream banner;
        banner << name << " yolo boxes=" << raw_parts.size();
        cv::rectangle(vis, cv::Point(0, 0), cv::Point(vis.cols, 28), cv::Scalar(0, 0, 0), cv::FILLED);
        cv::putText(vis, banner.str(), cv::Point(8, 20), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        const std::string path = vision_debug_record_path("head", "yolo");
        if (!path.empty() && cv::imwrite(path, vis))
        {
            std::cout << "[orch] 工件检测框 " << path << " 框数=" << raw_parts.size() << "\n";
            if (g_hw != nullptr)
                g_hw->last_debug_photo = path;
        }
    }
    out.parts = merge_overlapping_belt_parts(raw_parts);
    if (out.parts.size() != raw_parts.size())
        std::cout << "[orch] " << name << " 重叠框合成 " << raw_parts.size() << " → "
                  << out.parts.size() << "\n";
    out.ok = true;
    out.message = "ok";
    return out;
}

bool belt_part_in_grasp_window(const BeltPartMark &part)
{
    return part.x >= kBeltGraspWinXMinM && part.x <= kBeltGraspWinXMaxM &&
           part.y >= kBeltGraspWinYMinM && part.y <= kBeltGraspWinYMaxM;
}

void log_belt_used_box(const char *name, const char *role, int index, const BeltPartMark &part)
{
    const bool in = part.have_marker && belt_part_in_grasp_window(part);
    const double cx = 0.5 * (static_cast<double>(part.x0) + static_cast<double>(part.x1));
    const double cy = 0.5 * (static_cast<double>(part.y0) + static_cast<double>(part.y1));
    std::cout << std::fixed << "[orch] " << name << " " << role << " 检测框" << index
              << " " << belt_yolo_class_zh(part.class_id)
              << " conf=" << std::setprecision(3) << part.conf
              << " 像素框=(" << std::setprecision(0) << part.x0 << "," << part.y0 << ")-("
              << part.x1 << "," << part.y1 << ") 像素中心=(" << cx << "," << cy << ")";
    if (part.have_marker)
    {
        std::cout << std::setprecision(1) << " 码系 x=" << (part.x * 100.0)
                  << "cm y=" << (part.y * 100.0) << "cm z=" << (part.z * 100.0) << "cm "
                  << (in ? "在抓取窗口内" : "在抓取窗口外");
    }
    else
        std::cout << " 码系转换失败";
    std::cout << "\n";
}

BeltGraspWindow classify_belt_grasp_window(
    const std::vector<BeltPartMark> &parts, const char *tag)
{
    BeltGraspWindow out;
    const char *name = (tag && tag[0]) ? tag : "belt";
    for (int i = 0; i < static_cast<int>(parts.size()); ++i)
    {
        const BeltPartMark &part = parts[static_cast<size_t>(i)];
        log_belt_used_box(name, "夹取", i, part);
        if (!belt_part_in_grasp_window(part))
            continue;
        ++out.count;
    }
    if (out.count == 1)
    {
        out.right = !right_arm_motors_locked();
        out.left = right_arm_motors_locked();
    }
    else if (out.count >= 2)
    {
        out.right = !right_arm_motors_locked();
        out.left = true;
    }
    std::ostringstream line;
    line << std::fixed << std::setprecision(2) << "抓取窗口 x[" << kBeltGraspWinXMinM << ","
         << kBeltGraspWinXMaxM << "] y[" << kBeltGraspWinYMinM << "," << kBeltGraspWinYMaxM
         << "] 内 " << out.count << " 个";
    const std::string text = line.str();
    std::cout << "[orch] " << name << " " << text << "\n";
    board_note("info", text);
    return out;
}

std::vector<int> match_parts_to_queries(
    const std::vector<MarkerQuery> &queries,
    const std::vector<BeltPartMark> &parts,
    double tol_m)
{
    struct Cand
    {
        int q = 0;
        int p = 0;
        double d = 0;
    };
    std::vector<Cand> cands;
    for (int qi = 0; qi < static_cast<int>(queries.size()); ++qi)
    {
        for (int pi = 0; pi < static_cast<int>(parts.size()); ++pi)
        {
            const double dx = parts[static_cast<size_t>(pi)].x - queries[static_cast<size_t>(qi)].x;
            const double dy = parts[static_cast<size_t>(pi)].y - queries[static_cast<size_t>(qi)].y;
            if (std::abs(dx) > tol_m || std::abs(dy) > tol_m)
                continue;
            cands.push_back(Cand{qi, pi, std::hypot(dx, dy)});
        }
    }
    std::cout << std::fixed << std::setprecision(1);
    for (int qi = 0; qi < static_cast<int>(queries.size()); ++qi)
    {
        const MarkerQuery &q = queries[static_cast<size_t>(qi)];
        int best = -1;
        double best_d = 0;
        double best_dx = 0;
        double best_dy = 0;
        for (int pi = 0; pi < static_cast<int>(parts.size()); ++pi)
        {
            const double dx = parts[static_cast<size_t>(pi)].x - q.x;
            const double dy = parts[static_cast<size_t>(pi)].y - q.y;
            const double d = std::hypot(dx, dy);
            if (best >= 0 && d >= best_d)
                continue;
            best = pi;
            best_d = d;
            best_dx = dx;
            best_dy = dy;
        }
        std::ostringstream dev;
        dev << std::fixed << std::setprecision(1) << "最小偏差 " << q.name;
        if (best < 0)
            dev << " 无零件";
        else
        {
            const bool in_tol = std::abs(best_dx) <= tol_m && std::abs(best_dy) <= tol_m;
            dev << " ↔ 零件" << best << std::showpos
                << " dx=" << (best_dx * 100.0) << "cm"
                << " dy=" << (best_dy * 100.0) << "cm" << std::noshowpos
                << " 距离=" << (best_d * 100.0) << "cm"
                << (in_tol ? "" : " 超出门限");
        }
        const std::string line = dev.str();
        std::cout << "[orch] " << line << std::endl;
        board_note("info", line);
    }
    std::sort(cands.begin(), cands.end(), [](const Cand &a, const Cand &b) { return a.d < b.d; });
    std::vector<int> assigned(queries.size(), -1);
    std::vector<char> used(parts.size(), 0);
    for (const auto &cnd : cands)
    {
        if (assigned[static_cast<size_t>(cnd.q)] >= 0 || used[static_cast<size_t>(cnd.p)])
            continue;
        assigned[static_cast<size_t>(cnd.q)] = cnd.p;
        used[static_cast<size_t>(cnd.p)] = 1;
        std::cout << std::fixed << std::setprecision(3) << "[orch] 匹配 "
                  << queries[static_cast<size_t>(cnd.q)].name << " ↔ 零件" << cnd.p
                  << " 距离=" << cnd.d << " m\n";
    }
    return assigned;
}

namespace {
static int count_fallen_near_place_points(
    const ConveyorStationConfig &c, const char *tag, bool do_right, bool do_left)
{
    const char *name = (tag && tag[0]) ? tag : "belt";
    const bool check_right = do_right && !right_arm_motors_locked();
    const bool check_left = do_left;
    if (!check_right && !check_left)
        return 0;
    if (g_hw == nullptr || g_hw->pipeline == nullptr)
    {
        if (g_hw != nullptr)
            g_hw->last_belt_message = "无相机";
        return kVisionQrFail;
    }
    SegPoseBridge &bridge = g_hw->pipeline->bridge();
    if (!bridge.belt_engine_ready())
    {
        g_hw->last_belt_message = "传送带引擎未加载";
        return kVisionQrFail;
    }
    const int settle_ms = static_cast<int>(
        std::lround(std::max(0.0, c.grasp_photo_settle_sec) * 1000.0));
    std::cout << "[orch] " << name << " 放置完成，等 " << settle_ms
              << " ms 再拍，看放置点 ±6cm 内有没有倒件\n";
    if (settle_ms > 0)
        hardware_abort_sleep_ms(settle_ms);
    if (hardware_abort_requested())
        return -4;

    HeadBeltPerception perc = perceive_belt_head(
        g_hw->pipeline->cameras(), bridge, c.prefer_belt, kDebugVisualize, nullptr, nullptr, true);
    if (!perc.frame.ok)
    {
        g_hw->last_belt_message = "取帧失败";
        std::cerr << "[orch] " << name << " 放置确认取帧失败: " << perc.frame.message << "\n";
        return kVisionQrFail;
    }
    if (!perc.belt.ok || !perc.belt.in_robot)
    {
        g_hw->last_belt_message = "二维码解算失败";
        std::cerr << "[orch] " << name << " 放置确认二维码失败\n";
        return kVisionQrFail;
    }
    if(!apply_fixed_belt_origin_z(perc.belt, c, name))return kVisionQrFail;
    const BeltPartList parts = detect_belt_parts_full(
        bridge, perc.frame, c, perc.belt, name, perc.yolo_ran ? &perc.yolo : nullptr);
    if (!parts.ok)
    {
        g_hw->last_belt_message = parts.message;
        std::cerr << "[orch] " << name << " 放置确认检测失败: " << parts.message << "\n";
        return kVisionQrFail;
    }

    struct PlacePoint
    {
        double x = 0;
        double y = 0;
        const char *label = "";
    };
    std::vector<PlacePoint> points;
    if (check_right)
        points.push_back(PlacePoint{c.offset_right.x, c.offset_right.y, "右手放置点"});
    if (check_left)
        points.push_back(PlacePoint{c.offset_left.x, c.offset_left.y, "左手放置点"});

    constexpr double kPlaceTolM = 0.06;
    int fallen = 0;
    for (int i = 0; i < static_cast<int>(parts.parts.size()); ++i)
    {
        const BeltPartMark &part = parts.parts[static_cast<size_t>(i)];
        if (!part.have_marker)
            continue;
        bool in_region = false;
        for (const PlacePoint &pt : points)
        {
            const double dx = part.x - pt.x;
            const double dy = part.y - pt.y;
            if (std::abs(dx) > kPlaceTolM || std::abs(dy) > kPlaceTolM)
                continue;
            in_region = true;
            std::cout << std::fixed << std::setprecision(1) << "[orch] " << name
                      << " 放置确认 零件" << i << " " << belt_yolo_class_zh(part.class_id)
                      << " ↔ " << pt.label << std::showpos
                      << " dx=" << (dx * 100.0) << "cm dy=" << (dy * 100.0) << "cm"
                      << std::noshowpos << "\n";
        }
        if (!in_region || part.class_id != kFallenClassId)
            continue;
        ++fallen;
    }
    std::cout << "[orch] " << name << " 放置确认：放置点 ±6cm 内倒件 " << fallen << " 个\n";
    return fallen;
}
}

int confirm_belt_place_not_fallen(
    DispatchClient &client, const ConveyorStationConfig &c, const char *tag,
    bool do_right, bool do_left, const char *place_what)
{
    int vision_retry = 1;
    for (;;)
    {
        const int fallen = count_fallen_near_place_points(c, tag, do_right, do_left);
        if (fallen == -4)
            return -4;
        if (fallen == kVisionQrFail)
        {
            const int again = dispatch_belt_vision_again(client, vision_retry, place_what);
            if (again != 0)
                return again;
            continue;
        }
        if (fallen > 0)
        {
            std::cout << "[dispatch] " << place_what << " 放置点 ±6cm 内倒件 " << fallen
                      << " 个\n";
            const int choice = dispatch_report_ex(client, "工件倒了");
            if (choice != 0)
                return choice;
            std::cout << "[dispatch] " << place_what << " 倒件已解决，再拍一张确认\n";
            continue;
        }
        std::cout << "[dispatch] " << place_what << " 放置点没有倒件，继续\n";
        return 0;
    }
}
}
