#pragma once

#include <array>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "seg_pose_bridge.h"

constexpr int kPlaceGridRows = 6;
constexpr int kPlaceGridCols = 8;
constexpr int kPlaceGridCells = kPlaceGridRows * kPlaceGridCols;
constexpr int kPlaceGridNonAnchorCells = kPlaceGridCells - 4;

struct PlaceGridCellPub
{
    int row = -1;
    int col = -1;
    double corr_x = 0.0;
    double corr_y = 0.0;
    double raw_z = 0.0;
    int class_id = -1;
    bool has_point = false;
    bool is_anchor = false;
    bool placed = false;
};

/** 48 格放货阶段（由 place_non_anchor_first 与已放数量决定） */
enum class PlaceGridPlacePhase
{
    /** 配置=1：先放 44 非基准，corr xy */
    CorrNonAnchorFirst,
    /** 配置=1：44 满后，4 基准 raw xy */
    AnchorRawLast,
    /** 配置=0：先放 4 基准，corr xy */
    CorrAnchorFirst,
    /** 配置=0：4 基准满后，44 非基准 corr xy */
    CorrNonAnchorRest,
};

struct PlaceGridSession
{
    bool active = false;
    int placed_non_anchor = 0;
    int placed_anchor = 0;
    int num_anchors = 0;
    std::array<PlaceGridCellPub, kPlaceGridCells> cells{};
};

PlaceGridPlacePhase place_grid_place_phase(const PlaceGridSession &session);

const char *place_grid_place_phase_label(PlaceGridPlacePhase phase);

struct PlaceGridCorrectResult
{
    bool ok = false;
    std::string message;
    int num_detections = 0;
    int num_anchors = 0;
    std::string output_path;
};

/** 放货选格：poses[i] 对应 row_for_index[i]（棋盘排 0-based，不用 row_x_bounds） */
struct PlaceSlotCatalog
{
    std::vector<Eigen::Matrix<double, 1, 6>> poses;
    std::vector<int> row_for_index;
    std::vector<int> flat_cell;
};

void place_grid_session_reset();

const PlaceGridSession &place_grid_session();

bool place_grid_is_anchor_cell(int row_0, int col_0);

int place_grid_flat_index(int row_0, int col_0);

/** 头拍一次：拟合 48 格 corr，保留已放置标记，写 debug txt */
PlaceGridCorrectResult place_grid_session_update_from_pose_run(
    const PoseRunResult &detect,
    const std::array<double, 16> &cam2robot,
    const char *context_tag = "head_place");

/** 由 session 生成可放空位列表（corr 或基准 raw 阶段） */
bool place_grid_build_slot_catalog(
    const PlaceGridSession &session,
    const std::vector<Eigen::Matrix<double, 1, 6>> &raw_class1_holes,
    PlaceSlotCatalog &out);

void place_grid_mark_cell_placed(int flat_cell);

/** 兼容旧调用 */
PlaceGridCorrectResult place_grid_correct_write_from_pose_run(
    const PoseRunResult &detect,
    const std::array<double, 16> &cam2robot,
    const char *context_tag = "head_place");
