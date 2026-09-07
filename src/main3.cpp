#include "realsense_get.h"
#include "seg_pose_bridge.h"
#include "move_box_config.h"
#include "place_grid_correct.h"

#include <iostream>
#include <string>
#include <thread>

#include "head.h"
#include "gripper_interface.hpp"
#include "wt_box_tcp.h"
#include "reals_tcp.h"
#include "waist.h"
#include "Ti5_socketcan.h"
#include "Ti5_Arm.h"
#include "wt_client.h"
#include "lanxincontrol.h"
#include "tts_http_client.h"
#ifdef USE_TI5_BODY_CAN_DRIVER
#include "ti5_can_bridge.h"
#endif

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstdint>
#include <fstream>
#include <functional>
#include <filesystem>
#include <iomanip>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <numeric>
#include <optional>
#include <sstream>
#include <unistd.h>
#include <vector>

MoveBoxConfig g_move_cfg;

#include "move_box_runtime.h"

using namespace move_box;
using namespace row6_experiment;




int main() // 主要流程
{
    install_app_sigint_handler();
    std::string lanxin_ip = "192.168.100.201";
    int lanxin_port = 9000;

    LanxinControl ctrl;
    if (!ctrl.Connect(lanxin_ip, lanxin_port))
    {
        return 1;
    }

    if (!ctrl.SetTopoEndpoint(lanxin_ip, lanxin_port))
    {
        std::cerr << "[chassis] RPC 连接失败 " << lanxin_ip << ":" << lanxin_port << std::endl;
        return 1;
    }

    move_topo_until_ok(ctrl, "c01", 0);

    std::string cfg_err;
    if (!load_move_box_config(default_move_box_config_path(), g_move_cfg, cfg_err))
    {
        std::cerr << "[cfg] 加载失败，使用内置默认值: " << cfg_err << std::endl;
        g_move_cfg = default_move_box_config();
    }
    print_move_box_config(g_move_cfg);
    place_grid_session_reset();
    pose_vis_set_save_debug(kDebugVisualize);
    if (kDebugVisualize)
        std::cout << "[debug_vis] 识别调试图: " << project_root_dir()
                  << "/picture_debug/{head,right_hand,left_hand,head_place}/\n";
    std::cout << "[debug_vis] 对应原图: " << project_root_dir()
              << "/picture_debug/original/{head,right_hand,left_hand,head_place}/\n";

    gripper::Config cfg;
    cfg.grasp_torque_limit_nm = 4.5;
    cfg.soft_torque_limit_nm = 4.0;
    cfg.pos_filter = 0.5;
    cfg.soft_slew_rate = 100.0;
    cfg.soft_coast_margin_rad = 0.12;

    gripper::Gripper g(cfg);
    if (!g.start())
    {
        std::cerr << "夹爪启动失败\n";
        return 1;
    }

    PosePipeline pipeline;
    std::string err;
    if (!pipeline.init(err))
    {
        std::cerr << "初始化失败: " << err << std::endl;
        return 1;
    }

    RealSenseMultiCam &cameras = pipeline.cameras();
    SegPoseBridge &bridge = pipeline.bridge();

    // ---------- 腰部 / 待机位姿（与现场标定一致）----------
    Eigen::Matrix<double, 1, 6> posup_down_1_layer, posup_down_2_layer, posup_down_3_layer, yao_pos,
        put_down;

    posup_down_3_layer = g_move_cfg.waist.layer3_home;

    init_socketcan();
    hand_socketid_bind();

    {
        int waist_p[3] = {0, 40 * 65536 * 4 / 360, 0};
        uint32_t waist_canID[] = {30, 31, 32};
        socketcan_sendcommand(head_id, 3, waist_canID, 30, waist_p);
    }

    WaistRobot Ti5_waist;
    const std::string arm_yaml = project_root_dir() + "/config/Robot_Arm_Model.yaml";
    Robot_Arm Taihu_r("T7", "T170", "right", arm_yaml);
    Robot_Arm Taihu_l("T7", "T170", "left", arm_yaml);

    double waist_angles[1] = {0};
    uint32_t waist_canID[] = {1};
    smooth_motor_move_deg(waist_id, 1, waist_canID, waist_angles, 30.0);
    sleep(1);
    rev_motor_error(0);
    rev_motor_error(1);

    // sleep(8);

    // cout << Taihu_r.getJointPos() << endl;
    //  cout << Taihu_l.getJointPos() << endl;

    //  return 0;

    motor_speed_change();

    sleep(1);

    //  set_motor_current_zero();

    Ti5_waist.moveLToPos(posup_down_3_layer, 0.1);
    sleep(g_move_cfg.waist.move_settle_sec);

    Matrix<double, 1, 6> posr, posl, goal_last, goal_last_copy;

    posr = make_standby_pos_right();
    posl = make_standby_pos_left();

    bool need_topo_fang1 = true;

    {
        // 上电初始化：双臂先到待机位，首次拓扑 fang1 并转 -90°
        start_pos(Taihu_r, Taihu_l);
        arm_dual_line_move(Taihu_r, posr, Taihu_l, posl, 0.2);

        if (!prepare_chassis_for_grasp(ctrl, Ti5_waist, posup_down_3_layer, need_topo_fang1))
            return 1;
    }

    while (true) // 搬箱主循环：待机 → 识别 → 抓取 → 放货
    {
        static int s_main_loop_round = 0;
        ++s_main_loop_round;
        log_cycle_banner(s_main_loop_round);

        ensure_waist_layer3_x_home(Ti5_waist, posup_down_3_layer);

        // 每轮开始：双臂待机 → 查电；已在 fang1 时仅 -90/+90 抓放循环，不再重复拓扑
        arm_dual_line_move(Taihu_r, posr, Taihu_l, posl, 0.2);

        if (!ensure_charged_and_at_work_site(ctrl, need_topo_fang1))
        {
            sleep(1);
            continue;
        }

        if (!prepare_chassis_for_grasp(ctrl, Ti5_waist, posup_down_3_layer, need_topo_fang1))
        {
            sleep(1);
            continue;
        }

        std::array<double, 16> cam2robot{};
        if (!load_cam2robot_matrix(default_camera_to_robot_yaml_path(), cam2robot, err))
        {
            std::cerr << "读取 cam2robot 失败: " << err << std::endl;
        }
        else
        {
            GraspToStandbyResult gr = run_grasp_to_standby(
                g,
                Taihu_r,
                Taihu_l,
                Ti5_waist,
                posup_down_3_layer,
                cameras,
                bridge,
                cam2robot,
                err);

            if (gr.status == GraspToStandbyStatus::Ok)
            {
                bool waist_adjusted = gr.waist_adjusted;
                const bool place_ok = run_post_grasp_place_flow(
                    g,
                    Taihu_r,
                    Taihu_l,
                    gr.st,
                    gr.grasped_r,
                    gr.grasped_l,
                    Ti5_waist,
                    posup_down_3_layer,
                    ctrl,
                    cameras,
                    bridge,
                    cam2robot,
                    waist_adjusted);
                if (place_ok)
                    cout << "[arm] 放货完成，双臂回待机位，准备下一轮\n";
                else
                    cout << "[arm] 放货未完成，双臂回待机位\n";
                arm_dual_line_move(Taihu_r, posr, Taihu_l, posl, 0.2);
                sleep(1);
            }
            continue;
        }

        sleep(2); // 每轮间隔
    }

    std::cout << "\n已退出\n";
    g.stop();
    pipeline.shutdown(kDebugVisualize);
    return 0;
}
