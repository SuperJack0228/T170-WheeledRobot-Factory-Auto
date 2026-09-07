#include "place_grid_correct.h"

#include "function.h"
#include "move_box_config.h"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

namespace
{

constexpr int kGridRows = 6;
constexpr int kGridCols = 8;
constexpr int kGridCells = kGridRows * kGridCols;

/** 排方向间距、列方向间距 (m)，机器人 base 系 */
constexpr double kPitchX = 0.10;
constexpr double kPitchY = 0.10;

/** 同一格重复检测合并半径 (m) */
constexpr double kDedupeDistM = 0.03;

/** 自适应分排时相邻点 x 差超过此值视为换排 */
constexpr double kRowSplitGapMinM = 0.05;

/** 第2、3排(1-based) 从右往左(y低→高)第4、5格 → 0-based row=1,2 col=3,4 */
constexpr int kAnchorRows[2] = {1, 2};
constexpr int kAnchorCols[2] = {3, 4};

struct DetPoint
{
    int class_id = -1;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    int src_index = -1;
};

Eigen::Matrix<double, 4, 4> row_major_pose_to_matrix4(const std::array<double, 16> &pose)
{
    Eigen::Matrix<double, 4, 4> T;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            T(r, c) = pose[static_cast<size_t>(r * 4 + c)];
    return T;
}

struct GridCell
{
    int row = -1;
    int col = -1;
    int class_id = -1; /** -1=未知/补格 */
    bool has_raw = false;
    double raw_x = 0.0;
    double raw_y = 0.0;
    double raw_z = 0.0;
    double corr_x = 0.0;
    double corr_y = 0.0;
    int raw_count = 0;
    bool inferred_class = false;
    std::string flags;
};

Eigen::Matrix<double, 1, 6> pose_target_to_robot6(
    const std::array<double, 16> &cam2robot,
    const PoseTargetResult &t)
{
    const std::array<double, 16> pose_robot = transform_pose_cam_to_robot(cam2robot, t.pose_4x4);
    return T2PosEulerAngles(row_major_pose_to_matrix4(pose_robot));
}

std::vector<DetPoint> extract_class01_detections(
    const PoseRunResult &detect,
    const std::array<double, 16> &cam2robot)
{
    std::vector<DetPoint> out;
    out.reserve(detect.targets.size());
    for (size_t i = 0; i < detect.targets.size(); ++i)
    {
        const PoseTargetResult &t = detect.targets[i];
        if (!t.success)
            continue;
        if (t.class_id != kGraspDetectClassId && t.class_id != kPlaceDetectClassId)
            continue;
        const Eigen::Matrix<double, 1, 6> p = pose_target_to_robot6(cam2robot, t);
        out.push_back({t.class_id, p(0), p(1), p(2), static_cast<int>(i)});
    }
    return out;
}

struct DetCluster
{
    double sum_x = 0.0;
    double sum_y = 0.0;
    int count = 0;
    int class_id = kGraspDetectClassId;
    std::vector<DetPoint> members;
};

/** 同一位置（xy 差 < 3cm）的多个检测合并为一个点；有 class1 则记 class1 */
std::vector<DetPoint> dedupe_detections(
    const std::vector<DetPoint> &dets,
    std::vector<DetPoint> &dropped_dupes)
{
    dropped_dupes.clear();
    const double r2 = kDedupeDistM * kDedupeDistM;
    std::vector<DetCluster> clusters;
    clusters.reserve(dets.size());

    for (const DetPoint &p : dets)
    {
        int hit = -1;
        for (size_t ci = 0; ci < clusters.size(); ++ci)
        {
            const DetCluster &cl = clusters[ci];
            const double cx = cl.sum_x / static_cast<double>(cl.count);
            const double cy = cl.sum_y / static_cast<double>(cl.count);
            const double dx = p.x - cx;
            const double dy = p.y - cy;
            if (dx * dx + dy * dy > r2)
                continue;
            hit = static_cast<int>(ci);
            break;
        }
        if (hit < 0)
        {
            DetCluster cl;
            cl.sum_x = p.x;
            cl.sum_y = p.y;
            cl.count = 1;
            cl.class_id = p.class_id;
            cl.members.push_back(p);
            clusters.push_back(std::move(cl));
            continue;
        }
        DetCluster &cl = clusters[static_cast<size_t>(hit)];
        dropped_dupes.push_back(p);
        cl.sum_x += p.x;
        cl.sum_y += p.y;
        cl.count += 1;
        cl.members.push_back(p);
        if (p.class_id == kPlaceDetectClassId)
            cl.class_id = kPlaceDetectClassId;
    }

    std::vector<DetPoint> kept;
    kept.reserve(clusters.size());
    for (const DetCluster &cl : clusters)
    {
        DetPoint out;
        out.x = cl.sum_x / static_cast<double>(cl.count);
        out.y = cl.sum_y / static_cast<double>(cl.count);
        out.class_id = cl.class_id;
        out.src_index = cl.members.front().src_index;
        kept.push_back(out);
    }
    return kept;
}

bool column_assign_strictly_increasing(const std::vector<int> &cols)
{
    for (size_t i = 1; i < cols.size(); ++i)
    {
        if (cols[i] <= cols[i - 1])
            return false;
    }
    return !cols.empty();
}

/** 连续列 fallback：y 相同/DP 无解时强制 c0,c0+1,... */
std::vector<int> assign_columns_sequential(
    const std::vector<DetPoint> &row_pts,
    int n)
{
    std::vector<int> best_assign(static_cast<size_t>(n), 0);
    double best_cost = std::numeric_limits<double>::max();
    for (int c0 = 0; c0 <= kGridCols - n; ++c0)
    {
        double cost = 0.0;
        for (int i = 0; i < n; ++i)
        {
            const double expected_y =
                row_pts[0].y + static_cast<double>(c0 + i - 0) * kPitchY;
            cost += std::abs(row_pts[static_cast<size_t>(i)].y - expected_y);
        }
        if (cost < best_cost)
        {
            best_cost = cost;
            for (int i = 0; i < n; ++i)
                best_assign[static_cast<size_t>(i)] = c0 + i;
        }
    }
    return best_assign;
}

/** 一行内：保序 DP 将 n 个点分到 8 列，允许 y 间距不匀(零件歪) */
std::vector<int> assign_columns_in_row(const std::vector<DetPoint> &row_pts)
{
    const int n = static_cast<int>(row_pts.size());
    std::vector<int> empty;
    if (n <= 0)
        return empty;
    if (n > kGridCols)
        return empty;

    if (n == kGridCols)
        return assign_columns_sequential(row_pts, n);

    double best_cost = std::numeric_limits<double>::max();
    std::vector<int> best_assign(static_cast<size_t>(n), 0);

    for (int c0 = 0; c0 <= kGridCols - n; ++c0)
    {
        const double y_origin = row_pts[0].y - static_cast<double>(c0) * kPitchY;
        std::vector<std::vector<double>> dp(
            static_cast<size_t>(n),
            std::vector<double>(kGridCols, std::numeric_limits<double>::max()));
        std::vector<std::vector<int>> prev(
            static_cast<size_t>(n), std::vector<int>(kGridCols, -1));

        for (int c = c0; c < kGridCols; ++c)
        {
            const double expected_y = y_origin + static_cast<double>(c) * kPitchY;
            dp[0][static_cast<size_t>(c)] = std::abs(row_pts[0].y - expected_y);
        }

        for (int i = 1; i < n; ++i)
        {
            for (int c = c0 + (n - 1 - i); c < kGridCols; ++c)
            {
                const double expected_y = y_origin + static_cast<double>(c) * kPitchY;
                const double self = std::abs(row_pts[static_cast<size_t>(i)].y - expected_y);
                for (int pc = c0; pc < c; ++pc)
                {
                    if (dp[static_cast<size_t>(i - 1)][static_cast<size_t>(pc)] >=
                        std::numeric_limits<double>::max() * 0.5)
                        continue;
                    const double cost =
                        dp[static_cast<size_t>(i - 1)][static_cast<size_t>(pc)] + self;
                    if (cost < dp[static_cast<size_t>(i)][static_cast<size_t>(c)])
                    {
                        dp[static_cast<size_t>(i)][static_cast<size_t>(c)] = cost;
                        prev[static_cast<size_t>(i)][static_cast<size_t>(c)] = pc;
                    }
                }
            }
        }

        for (int c = c0 + n - 1; c < kGridCols; ++c)
        {
            if (dp[static_cast<size_t>(n - 1)][static_cast<size_t>(c)] >= best_cost)
                continue;
            best_cost = dp[static_cast<size_t>(n - 1)][static_cast<size_t>(c)];
            std::vector<int> assign(static_cast<size_t>(n));
            int cur = c;
            for (int i = n - 1; i >= 0; --i)
            {
                assign[static_cast<size_t>(i)] = cur;
                cur = prev[static_cast<size_t>(i)][static_cast<size_t>(cur)];
            }
            best_assign = assign;
        }
    }

    if (best_cost >= std::numeric_limits<double>::max() * 0.5 ||
        !column_assign_strictly_increasing(best_assign))
        return assign_columns_sequential(row_pts, n);
    return best_assign;
}

/** 按 x 从小到大切 6 排（纯几何，不读配置文件） */
void split_sorted_x_into_rows(
    const std::vector<DetPoint> &sorted,
    std::array<std::vector<DetPoint>, kGridRows> &rows,
    int start_idx,
    int end_idx)
{
    const int count = end_idx - start_idx;
    if (count <= 0)
        return;
    int idx = start_idx;
    for (int r = 0; r < kGridRows; ++r)
    {
        const int rows_left = kGridRows - r;
        const int pts_left = end_idx - idx;
        if (pts_left <= 0)
            break;
        const int take = std::min(kGridCols, (pts_left + rows_left - 1) / rows_left);
        for (int j = 0; j < take && idx < end_idx; ++j, ++idx)
            rows[static_cast<size_t>(r)].push_back(sorted[static_cast<size_t>(idx)]);
    }
}

/** 48 点：x 排序后每 8 个一排；否则先找 x 大间隙切 6 排，再不行均匀切 */
std::array<std::vector<DetPoint>, kGridRows> split_detections_into_rows(
    const std::vector<DetPoint> &dets)
{
    std::array<std::vector<DetPoint>, kGridRows> rows;
    if (dets.empty())
        return rows;

    std::vector<DetPoint> sorted = dets;
    std::sort(sorted.begin(), sorted.end(), [](const DetPoint &a, const DetPoint &b)
              { return a.x < b.x; });

    if (static_cast<int>(sorted.size()) == kGridCells)
    {
        for (int i = 0; i < kGridCells; ++i)
            rows[static_cast<size_t>(i / kGridCols)].push_back(sorted[static_cast<size_t>(i)]);
        return rows;
    }

    struct Gap
    {
        double dx = 0.0;
        int after = -1;
    };
    std::vector<Gap> gaps;
    gaps.reserve(sorted.size());
    for (size_t i = 0; i + 1 < sorted.size(); ++i)
    {
        const double dx = sorted[i + 1].x - sorted[i].x;
        if (dx >= kRowSplitGapMinM)
            gaps.push_back({dx, static_cast<int>(i)});
    }
    std::sort(gaps.begin(), gaps.end(), [](const Gap &a, const Gap &b)
              { return a.dx > b.dx; });

    std::vector<int> split_after;
    const int need_splits = kGridRows - 1;
    for (int i = 0; i < need_splits && i < static_cast<int>(gaps.size()); ++i)
        split_after.push_back(gaps[static_cast<size_t>(i)].after);
    std::sort(split_after.begin(), split_after.end());

    if (static_cast<int>(split_after.size()) == need_splits)
    {
        int start = 0;
        for (int r = 0; r < kGridRows; ++r)
        {
            const int end = (r < need_splits)
                                ? split_after[static_cast<size_t>(r)] + 1
                                : static_cast<int>(sorted.size());
            for (int i = start; i < end; ++i)
                rows[static_cast<size_t>(r)].push_back(sorted[static_cast<size_t>(i)]);
            start = end;
        }
        return rows;
    }

    split_sorted_x_into_rows(sorted, rows, 0, static_cast<int>(sorted.size()));
    return rows;
}

void merge_into_cell(GridCell &cell, const DetPoint &p)
{
    if (!cell.has_raw)
    {
        cell.has_raw = true;
        cell.raw_x = p.x;
        cell.raw_y = p.y;
        cell.raw_z = p.z;
        cell.class_id = p.class_id;
        cell.raw_count = 1;
        return;
    }
    cell.raw_x = (cell.raw_x * cell.raw_count + p.x) / (cell.raw_count + 1);
    cell.raw_y = (cell.raw_y * cell.raw_count + p.y) / (cell.raw_count + 1);
    cell.raw_z = (cell.raw_z * cell.raw_count + p.z) / (cell.raw_count + 1);
    cell.raw_count += 1;
    if (p.class_id == kPlaceDetectClassId)
        cell.class_id = kPlaceDetectClassId;
    if (!cell.flags.empty())
        cell.flags += ";";
    cell.flags += "multi_raw";
}

void assign_detections_to_grid(
    const std::vector<DetPoint> &dets,
    std::array<GridCell, kGridCells> &grid,
    std::vector<DetPoint> &dropped)
{
    dropped.clear();
    for (int r = 0; r < kGridRows; ++r)
    {
        for (int c = 0; c < kGridCols; ++c)
        {
            grid[r * kGridCols + c].row = r;
            grid[r * kGridCols + c].col = c;
        }
    }

    const std::array<std::vector<DetPoint>, kGridRows> by_row =
        split_detections_into_rows(dets);

    for (int row = 0; row < kGridRows; ++row)
    {
        std::vector<DetPoint> row_pts = by_row[static_cast<size_t>(row)];
        if (row_pts.empty())
            continue;
        std::sort(row_pts.begin(), row_pts.end(), [](const DetPoint &a, const DetPoint &b)
                  { return a.y < b.y; });

        if (static_cast<int>(row_pts.size()) > kGridCols)
        {
            /** 一行超过 8 个：按 y 均匀抽 8 个代表，其余记 dropped */
            const int n = static_cast<int>(row_pts.size());
            std::vector<int> pick(static_cast<size_t>(kGridCols));
            for (int j = 0; j < kGridCols; ++j)
                pick[static_cast<size_t>(j)] = (j * (n - 1)) / (kGridCols - 1);

            std::vector<DetPoint> subset;
            subset.reserve(kGridCols);
            for (int j = 0; j < kGridCols; ++j)
            {
                const int idx = pick[static_cast<size_t>(j)];
                subset.push_back(row_pts[static_cast<size_t>(idx)]);
            }
            for (int i = 0; i < n; ++i)
            {
                bool is_pick = false;
                for (int idx : pick)
                {
                    if (idx == i)
                    {
                        is_pick = true;
                        break;
                    }
                }
                if (!is_pick)
                    dropped.push_back(row_pts[static_cast<size_t>(i)]);
            }
            row_pts = std::move(subset);
        }

        const std::vector<int> cols = assign_columns_in_row(row_pts);
        for (size_t i = 0; i < row_pts.size() && i < cols.size(); ++i)
        {
            const int col = cols[i];
            if (col < 0 || col >= kGridCols)
            {
                dropped.push_back(row_pts[i]);
                continue;
            }
            merge_into_cell(grid[static_cast<size_t>(row * kGridCols + col)], row_pts[i]);
        }
    }
}

int count_anchors(const std::array<GridCell, kGridCells> &grid)
{
    int n = 0;
    for (int ar : kAnchorRows)
    {
        for (int ac : kAnchorCols)
        {
            const GridCell &cell = grid[static_cast<size_t>(ar * kGridCols + ac)];
            if (cell.has_raw && cell.class_id == kPlaceDetectClassId)
                ++n;
        }
    }
    return n;
}

void apply_anchor_correction(std::array<GridCell, kGridCells> &grid)
{
    struct Anchor
    {
        int row;
        int col;
        double x;
        double y;
    };
    std::vector<Anchor> anchors;
    anchors.reserve(4);
    for (int ar : kAnchorRows)
    {
        for (int ac : kAnchorCols)
        {
            GridCell &cell = grid[static_cast<size_t>(ar * kGridCols + ac)];
            if (!cell.has_raw || cell.class_id != kPlaceDetectClassId)
                continue;
            anchors.push_back({ar, ac, cell.raw_x, cell.raw_y});
        }
    }

    for (int r = 0; r < kGridRows; ++r)
    {
        for (int c = 0; c < kGridCols; ++c)
        {
            GridCell &cell = grid[static_cast<size_t>(r * kGridCols + c)];
            if (anchors.empty())
            {
                if (cell.has_raw)
                {
                    cell.corr_x = cell.raw_x;
                    cell.corr_y = cell.raw_y;
                }
                continue;
            }

            double sum_x = 0.0;
            double sum_y = 0.0;
            int cnt = 0;
            for (const Anchor &a : anchors)
            {
                sum_x += a.x + static_cast<double>(r - a.row) * kPitchX;
                sum_y += a.y + static_cast<double>(c - a.col) * kPitchY;
                ++cnt;
            }
            cell.corr_x = sum_x / static_cast<double>(cnt);
            cell.corr_y = sum_y / static_cast<double>(cnt);
        }
    }
}

void infer_missing_classes(std::array<GridCell, kGridCells> &grid)
{
    for (GridCell &cell : grid)
    {
        if (cell.has_raw)
            continue;
        cell.class_id = -1;
        cell.inferred_class = true;
    }
}

std::string timestamp_for_filename()
{
    using clock = std::chrono::system_clock;
    const auto now = clock::now();
    const std::time_t t = clock::to_time_t(now);
    std::tm tm_local{};
    localtime_r(&t, &tm_local);
    std::ostringstream oss;
    oss << std::put_time(&tm_local, "%Y%m%d_%H%M%S");
    return oss.str();
}

std::string place_grid_correct_output_path()
{
    return project_root_dir() + "/picture_debug/place_grid_correct.txt";
}

bool write_grid_correct_txt(
    const std::array<GridCell, kGridCells> &grid,
    const std::vector<DetPoint> &dets,
    int num_anchors,
    const char *context_tag,
    const std::vector<DetPoint> &dropped_dupes,
    const std::vector<DetPoint> &dropped_extra)
{
    const std::string path = place_grid_correct_output_path();
    std::filesystem::create_directories(
        std::filesystem::path(path).parent_path());

    std::ofstream ofs(path, std::ios::app);
    if (!ofs)
    {
        std::cerr << "[grid_correct] 无法写入 " << path << "\n";
        return false;
    }

    ofs << "================================================================================\n";
    ofs << "# place_grid_correct " << timestamp_for_filename()
        << " context=" << (context_tag ? context_tag : "") << "\n";
    ofs << "# robot_base_xy pitch_x=" << kPitchX << " pitch_y=" << kPitchY
        << " grid=" << kGridRows << "x" << kGridCols << "\n";
    ofs << "# detections_class01=" << dets.size()
        << " deduped_dupes=" << dropped_dupes.size()
        << " dropped_extra=" << dropped_extra.size()
        << " anchors_class1=" << num_anchors
        << " (anchor rows 2,3 cols 4,5 from right=y low)\n";
    ofs << "# row col class raw_x raw_y corr_x corr_y flags\n";

    for (int r = 0; r < kGridRows; ++r)
    {
        for (int c = 0; c < kGridCols; ++c)
        {
            const GridCell &cell = grid[static_cast<size_t>(r * kGridCols + c)];
            ofs << std::fixed << std::setprecision(4);
            ofs << r + 1 << ' ' << c + 1 << ' ';
            if (cell.class_id < 0)
                ofs << '?';
            else
                ofs << cell.class_id;
            ofs << ' ';
            if (cell.has_raw)
                ofs << cell.raw_x << ' ' << cell.raw_y;
            else
                ofs << "nan nan";
            ofs << ' ' << cell.corr_x << ' ' << cell.corr_y;
            if (!cell.flags.empty())
                ofs << ' ' << cell.flags;
            ofs << '\n';
        }
    }

    ofs << "# dropped_or_unassigned_detections (if any)\n";
    for (const DetPoint &p : dropped_dupes)
        ofs << "dup class=" << p.class_id << ' ' << p.x << ' ' << p.y << '\n';
    for (const DetPoint &p : dropped_extra)
        ofs << "extra class=" << p.class_id << ' ' << p.x << ' ' << p.y << '\n';
    ofs << std::flush;
    std::cout << "[grid_correct] 已写入 " << path << " det=" << dets.size()
              << " anchors=" << num_anchors << "\n";
    return true;
}

} // namespace

namespace
{

PlaceGridSession g_place_grid_session;

bool internal_cell_is_anchor(int row_0, int col_0)
{
    for (int ar : kAnchorRows)
    {
        for (int ac : kAnchorCols)
        {
            if (row_0 == ar && col_0 == ac)
                return true;
        }
    }
    return false;
}

void sync_public_cell(PlaceGridCellPub &pub, const GridCell &cell, bool keep_placed)
{
    const bool was_placed = keep_placed && pub.placed;
    pub.row = cell.row;
    pub.col = cell.col;
    pub.corr_x = cell.corr_x;
    pub.corr_y = cell.corr_y;
    pub.raw_z = cell.raw_z;
    pub.class_id = cell.class_id;
    pub.has_point = cell.has_raw;
    pub.is_anchor = internal_cell_is_anchor(cell.row, cell.col);
    pub.placed = was_placed;
}

int count_placed_non_anchor(const PlaceGridSession &s)
{
    int n = 0;
    for (const PlaceGridCellPub &c : s.cells)
    {
        if (c.placed && !c.is_anchor)
            ++n;
    }
    return n;
}

int count_placed_anchor(const PlaceGridSession &s)
{
    int n = 0;
    for (const PlaceGridCellPub &c : s.cells)
    {
        if (c.placed && c.is_anchor)
            ++n;
    }
    return n;
}

void append_corr_slots(
    const PlaceGridSession &session,
    PlaceSlotCatalog &out,
    double default_z,
    bool anchors_only,
    bool non_anchor_only)
{
    for (int r = 0; r < kPlaceGridRows; ++r)
    {
        for (int c = 0; c < kPlaceGridCols; ++c)
        {
            const int flat = place_grid_flat_index(r, c);
            const PlaceGridCellPub &cell =
                session.cells[static_cast<size_t>(flat)];
            if (cell.placed)
                continue;
            if (anchors_only && !cell.is_anchor)
                continue;
            if (non_anchor_only && cell.is_anchor)
                continue;
            if (cell.class_id != kPlaceDetectClassId)
                continue;
            Eigen::Matrix<double, 1, 6> pose;
            pose.setZero();
            pose(0) = cell.corr_x;
            pose(1) = cell.corr_y;
            pose(2) = cell.has_point ? cell.raw_z : default_z;
            out.poses.push_back(pose);
            out.row_for_index.push_back(r);
            out.flat_cell.push_back(flat);
        }
    }
}

double median_z_from_holes(const std::vector<Eigen::Matrix<double, 1, 6>> &holes)
{
    if (holes.empty())
        return -0.29;
    std::vector<double> zs;
    zs.reserve(holes.size());
    for (const auto &p : holes)
        zs.push_back(p(2));
    const size_t mid = zs.size() / 2;
    std::nth_element(zs.begin(), zs.begin() + static_cast<std::ptrdiff_t>(mid), zs.end());
    return zs[mid];
}

/** 基准格 raw 阶段：x 小两枚→第2排，x 大两枚→第3排；行内 y 小→右数第4格 */
void assign_anchor_raw_xy(
    std::vector<Eigen::Matrix<double, 1, 6>> holes,
    std::array<std::optional<Eigen::Matrix<double, 1, 6>>, kPlaceGridCells> &anchor_pose)
{
    if (holes.size() < 4)
        return;
    if (holes.size() > 4)
    {
        std::sort(holes.begin(), holes.end(), [](const auto &a, const auto &b)
                  { return a(0) < b(0); });
        holes.resize(4);
    }
    std::sort(holes.begin(), holes.end(), [](const auto &a, const auto &b)
              { return a(0) < b(0); });
    for (int half = 0; half < 2; ++half)
    {
        std::array<Eigen::Matrix<double, 1, 6>, 2> pair = {
            holes[static_cast<size_t>(half * 2)],
            holes[static_cast<size_t>(half * 2 + 1)]};
        std::sort(pair.begin(), pair.end(), [](const auto &a, const auto &b)
                  { return a(1) < b(1); });
        const int row_0 = 1 + half;
        anchor_pose[static_cast<size_t>(row_0 * kGridCols + 3)] = pair[0];
        anchor_pose[static_cast<size_t>(row_0 * kGridCols + 4)] = pair[1];
    }
}

PlaceGridCorrectResult build_grid_from_detect(
    const PoseRunResult &detect,
    const std::array<double, 16> &cam2robot,
    std::array<GridCell, kGridCells> &grid,
    std::vector<DetPoint> &dets_out,
    std::vector<DetPoint> &dropped_dupes_out,
    std::vector<DetPoint> &dropped_extra_out,
    int &num_anchors_out)
{
    PlaceGridCorrectResult out;
    dropped_dupes_out.clear();
    dropped_extra_out.clear();
    if (!detect.ok)
    {
        out.message = "算法未成功，跳过棋盘校正";
        return out;
    }

    const std::vector<DetPoint> raw_dets =
        extract_class01_detections(detect, cam2robot);
    dets_out = dedupe_detections(raw_dets, dropped_dupes_out);
    out.num_detections = static_cast<int>(dets_out.size());

    assign_detections_to_grid(dets_out, grid, dropped_extra_out);
    infer_missing_classes(grid);

    num_anchors_out = count_anchors(grid);
    out.num_anchors = num_anchors_out;
    apply_anchor_correction(grid);

    out.ok = num_anchors_out >= 4;
    if (!out.ok)
        out.message = "基准点不足 anchors=" + std::to_string(num_anchors_out);
    else
        out.message = "棋盘 OK det=" + std::to_string(out.num_detections) +
                      " anchors=" + std::to_string(num_anchors_out);
    return out;
}

} // namespace

PlaceGridPlacePhase place_grid_place_phase(const PlaceGridSession &session)
{
    const bool non_anchor_first = g_move_cfg.place.place_non_anchor_first != 0;
    if (non_anchor_first)
    {
        if (session.placed_non_anchor >= kPlaceGridNonAnchorCells)
            return PlaceGridPlacePhase::AnchorRawLast;
        return PlaceGridPlacePhase::CorrNonAnchorFirst;
    }
    if (session.placed_anchor < 4)
        return PlaceGridPlacePhase::CorrAnchorFirst;
    return PlaceGridPlacePhase::CorrNonAnchorRest;
}

const char *place_grid_place_phase_label(const PlaceGridPlacePhase phase)
{
    switch (phase)
    {
    case PlaceGridPlacePhase::CorrNonAnchorFirst:
        return "corr44";
    case PlaceGridPlacePhase::AnchorRawLast:
        return "raw4";
    case PlaceGridPlacePhase::CorrAnchorFirst:
        return "corr4";
    case PlaceGridPlacePhase::CorrNonAnchorRest:
        return "corr44rest";
    }
    return "?";
}

void place_grid_session_reset()
{
    g_place_grid_session = PlaceGridSession{};
    for (int r = 0; r < kPlaceGridRows; ++r)
    {
        for (int c = 0; c < kPlaceGridCols; ++c)
        {
            PlaceGridCellPub &cell =
                g_place_grid_session.cells[static_cast<size_t>(r * kPlaceGridCols + c)];
            cell.row = r;
            cell.col = c;
            cell.is_anchor = place_grid_is_anchor_cell(r, c);
            cell.placed = false;
        }
    }
    std::cout << "[grid_place] 48格放置状态已清零\n";
}

const PlaceGridSession &place_grid_session()
{
    return g_place_grid_session;
}

bool place_grid_is_anchor_cell(int row_0, int col_0)
{
    return internal_cell_is_anchor(row_0, col_0);
}

int place_grid_flat_index(int row_0, int col_0)
{
    if (row_0 < 0 || row_0 >= kPlaceGridRows || col_0 < 0 || col_0 >= kPlaceGridCols)
        return -1;
    return row_0 * kPlaceGridCols + col_0;
}

void place_grid_mark_cell_placed(int flat_cell)
{
    if (flat_cell < 0 || flat_cell >= kPlaceGridCells)
        return;
    PlaceGridCellPub &cell =
        g_place_grid_session.cells[static_cast<size_t>(flat_cell)];
    if (cell.placed)
        return;
    cell.placed = true;
    g_place_grid_session.placed_non_anchor = count_placed_non_anchor(g_place_grid_session);
    g_place_grid_session.placed_anchor = count_placed_anchor(g_place_grid_session);
    const PlaceGridPlacePhase phase = place_grid_place_phase(g_place_grid_session);
    std::cout << "[grid_place] 已放 排" << (cell.row + 1) << " 列" << (cell.col + 1)
              << (cell.is_anchor ? " [基准]" : "")
              << " 非基准=" << g_place_grid_session.placed_non_anchor << "/"
              << kPlaceGridNonAnchorCells << " 基准="
              << g_place_grid_session.placed_anchor << "/4"
              << " 下阶段=" << place_grid_place_phase_label(phase) << "\n";
}

PlaceGridCorrectResult place_grid_session_update_from_pose_run(
    const PoseRunResult &detect,
    const std::array<double, 16> &cam2robot,
    const char *context_tag)
{
    PlaceGridCorrectResult out;
    out.output_path = place_grid_correct_output_path();

    std::array<GridCell, kGridCells> grid{};
    std::vector<DetPoint> dets;
    std::vector<DetPoint> dropped_dupes;
    std::vector<DetPoint> dropped_extra;
    int num_anchors = 0;
    PlaceGridCorrectResult built = build_grid_from_detect(
        detect, cam2robot, grid, dets, dropped_dupes, dropped_extra, num_anchors);
    out.num_detections = built.num_detections;
    out.num_anchors = built.num_anchors;
    out.message = built.message;

    if (!built.ok)
    {
        std::cerr << "[grid_place] " << out.message << "，本帧不更新放货棋盘\n";
        return out;
    }

    for (int r = 0; r < kPlaceGridRows; ++r)
    {
        for (int c = 0; c < kPlaceGridCols; ++c)
        {
            const GridCell &src = grid[static_cast<size_t>(r * kPlaceGridCols + c)];
            PlaceGridCellPub &dst =
                g_place_grid_session.cells[static_cast<size_t>(r * kPlaceGridCols + c)];
            sync_public_cell(dst, src, true);
        }
    }

    g_place_grid_session.active = true;
    g_place_grid_session.num_anchors = num_anchors;
    g_place_grid_session.placed_non_anchor = count_placed_non_anchor(g_place_grid_session);
    g_place_grid_session.placed_anchor = count_placed_anchor(g_place_grid_session);

    write_grid_correct_txt(
        grid, dets, num_anchors, context_tag, dropped_dupes, dropped_extra);

    out.ok = true;
    const PlaceGridPlacePhase phase = place_grid_place_phase(g_place_grid_session);
    std::cout << "[grid_place] " << out.message << " 已放非基准="
              << g_place_grid_session.placed_non_anchor << "/"
              << kPlaceGridNonAnchorCells << " 基准="
              << g_place_grid_session.placed_anchor << "/4"
              << " 顺序=" << (g_move_cfg.place.place_non_anchor_first != 0 ? "先44" : "先4")
              << " 阶段=" << place_grid_place_phase_label(phase) << "\n";
    return out;
}

bool place_grid_build_slot_catalog(
    const PlaceGridSession &session,
    const std::vector<Eigen::Matrix<double, 1, 6>> &raw_class1_holes,
    PlaceSlotCatalog &out)
{
    out.poses.clear();
    out.row_for_index.clear();
    out.flat_cell.clear();
    if (!session.active)
        return false;

    const double default_z = median_z_from_holes(raw_class1_holes);
    const PlaceGridPlacePhase phase = place_grid_place_phase(session);

    if (phase == PlaceGridPlacePhase::AnchorRawLast)
    {
        std::vector<Eigen::Matrix<double, 1, 6>> anchor_holes = raw_class1_holes;
        double x_min = 1e9;
        double x_max = -1e9;
        for (int r = 0; r < kPlaceGridRows; ++r)
        {
            for (int c = 0; c < kPlaceGridCols; ++c)
            {
                if (!place_grid_is_anchor_cell(r, c))
                    continue;
                const PlaceGridCellPub &cell =
                    session.cells[static_cast<size_t>(place_grid_flat_index(r, c))];
                if (cell.placed)
                    continue;
                x_min = std::min(x_min, cell.corr_x);
                x_max = std::max(x_max, cell.corr_x);
            }
        }
        if (x_min < x_max && !anchor_holes.empty())
        {
            std::vector<Eigen::Matrix<double, 1, 6>> filtered;
            filtered.reserve(anchor_holes.size());
            for (const auto &h : anchor_holes)
            {
                if (h(0) >= x_min - 0.08 && h(0) <= x_max + 0.08)
                    filtered.push_back(h);
            }
            if (filtered.size() >= 4)
                anchor_holes = std::move(filtered);
        }

        std::array<std::optional<Eigen::Matrix<double, 1, 6>>, kPlaceGridCells> anchor_pose{};
        assign_anchor_raw_xy(anchor_holes, anchor_pose);
        for (int r = 0; r < kPlaceGridRows; ++r)
        {
            for (int c = 0; c < kPlaceGridCols; ++c)
            {
                if (!place_grid_is_anchor_cell(r, c))
                    continue;
                const int flat = place_grid_flat_index(r, c);
                const PlaceGridCellPub &cell =
                    session.cells[static_cast<size_t>(flat)];
                if (cell.placed)
                    continue;
                Eigen::Matrix<double, 1, 6> pose;
                pose.setZero();
                if (anchor_pose[static_cast<size_t>(flat)])
                    pose = *anchor_pose[static_cast<size_t>(flat)];
                else
                {
                    pose(0) = cell.corr_x;
                    pose(1) = cell.corr_y;
                    pose(2) = cell.has_point ? cell.raw_z : default_z;
                }
                out.poses.push_back(pose);
                out.row_for_index.push_back(r);
                out.flat_cell.push_back(flat);
            }
        }
        return !out.poses.empty();
    }

    if (phase == PlaceGridPlacePhase::CorrAnchorFirst)
    {
        append_corr_slots(session, out, default_z, true, false);
        return !out.poses.empty();
    }

    append_corr_slots(session, out, default_z, false, true);
    return !out.poses.empty();
}

PlaceGridCorrectResult place_grid_correct_write_from_pose_run(
    const PoseRunResult &detect,
    const std::array<double, 16> &cam2robot,
    const char *context_tag)
{
    return place_grid_session_update_from_pose_run(detect, cam2robot, context_tag);
}
