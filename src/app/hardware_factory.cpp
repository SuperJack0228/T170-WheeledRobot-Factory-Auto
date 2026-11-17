#include "internal/state.h"
#include "t170/hardware/device_bus.h"
#include "t170/vision/debug.h"
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <thread>

namespace t170::app::detail {
bool init_hardware(std::string &err)
{
    if (!load_move_box_config(default_move_box_config_path(), g_move_cfg, err))
        return false;
    std::cout << "[paths] root=" << project_root_dir()
              << " config=" << t170::paths::config_directory()
              << " logs=" << t170::paths::log_directory() << std::endl;
    print_move_box_config(g_move_cfg);
    vision_debug_init_from_env();
    pose_vis_set_save_debug(vision_debug_save_pairs());
    pose_vis_set_gui_enabled(vision_debug_gui());

    if (!init_socketcan())
    {
        err = device_scan_error();
        return false;
    }
    hand_socketid_bind();

    gripper::Config grip_cfg;
    grip_cfg.grasp_torque_limit_nm = 4.5;
    grip_cfg.soft_torque_limit_nm = 4.0;
    grip_cfg.pos_filter = 0.5;
    grip_cfg.soft_slew_rate = 100.0;
    grip_cfg.soft_coast_margin_rad = 0.12;
    grip_cfg.detect_retries = 2;
    try
    {
        g_grip = std::make_unique<gripper::Gripper>(grip_cfg);
        if (!g_grip->start())
        {
            err = "夹爪启动失败：一个夹爪都没有";
            return false;
        }
    }
    catch (const std::exception &ex)
    {
        err = std::string("夹爪初始化失败: ") + ex.what();
        return false;
    }

    g_pipeline = std::make_unique<PosePipeline>();
    if (!g_pipeline->init(err))
        return false;

    g_waist = std::make_unique<WaistRobot>();
    const std::string arm_yaml = t170::paths::config_file("Robot_Arm_Model.yaml").string();
    g_arm_r = std::make_unique<Robot_Arm>("T7", "T170", "right", arm_yaml);
    g_arm_l = std::make_unique<Robot_Arm>("T7", "T170", "left", arm_yaml);
    rev_motor_error(0);
    rev_motor_error(1);
    motor_speed_change();

    // 腰的 layer3 是绝对笛卡尔位姿，不能假设上电时它在 home：上一次进程可能把它停在别处，
    // 按错误的起点算出的"降腰"目标会把躯干移到非预期高度。这里只回读不运动。
    g_layer3 = g_move_cfg.waist.layer3_home;
    if (!move_box::sync_waist_layer3_from_encoders(*g_waist, g_layer3, "启动"))
    {
        g_layer3 = g_move_cfg.waist.layer3_home;
        std::cerr << "[waist] 警告：腰部位置未知，抓取流程中的腰部运动已禁用，请先执行 home\n";
    }
    g_session.g = g_grip.get();
    g_session.arm_r = g_arm_r.get();
    g_session.arm_l = g_arm_l.get();
    g_session.waist = g_waist.get();
    g_session.layer3 = &g_layer3;
    g_session.pipeline = g_pipeline.get();
    g_session.abort = &g_abort;
    orch::g_hw = &g_session;
    state().chassis=t170::hardware::make_seer_chassis(g_move_cfg.chassis.host);
    state().gateway=t170::integration::dispatch::make_robot_client_gateway();
    state().services=std::make_unique<t170::tasks::Services>(t170::tasks::Services{*state().chassis,*state().gateway});
    t170::tasks::bind_services(state().services.get());
    can_io_fault_clear();
    return true;
}
}
