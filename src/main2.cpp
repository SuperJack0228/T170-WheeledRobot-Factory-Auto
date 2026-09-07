#include "lanxincontrol.h"

#include <iostream>

#include "head.h"
#include "reals_tcp.h"
#include "waist.h"
#include "wt_client.h"
#include "Ti5_socketcan.h"
#include "wushi.h"
#include "shopclient.h"

int main()
{


    //  init_socketcan();
    // hand_socketid_bind();
    // rev_motor_error(0);
    // rev_motor_error(1);





    //  aoyi_init();
    // port_bind();

    // tiger_hand(right_a);
    // tiger_hand(left_a);



    //  int idhand_p0[3] = {0, 0, 0};
    // int idhand_p[3] = {0, 21845, 0};
    // int waist_p[3] = {0, -7281};
    // uint32_t hand_canID[] = {30, 31, 32};
    // uint32_t waist_canID[] = {1};

    // socketcan_sendcommand(head_id, 3, hand_canID, 30, idhand_p);
    // socketcan_sendcommand(waist_id, 1, waist_canID, 30, waist_p);

    // return 0;


    LanxinControl ctrl;

    // 连接：InitClient + GetAllHandleRobotTaskDev（等同 topo_move.cpp 12–35 行）
    if (!ctrl.SetTopoEndpoint("192.168.3.19", 9001))
    {
        return 1;
        cout << "lianjie shibai" << endl;
    }

     cout << "lianjie successs" << endl;
        

    // 仅发任务 + 等待（等同 topo_move.cpp 47–57 行），内部不再 InitClient
    string point;
    int a = 0;

    while (1)
    {
        // sleep(1);
        // if (a == 0)
        // {
        //     a=1;
        //     point = "Goal_LXOAJ";
        // }
        // else
        // {
        //     a=0;
        //     point = "Goal_AuLhj";
        // }
        cin >> point ;

        ctrl.MoveTopo(point, 0);
    }

    return 0;
}



int main545454()
{

    LanxinControl ctrl;
    std::string lanxin_ip = "192.168.100.201";
    int lanxin_port = 9000;
    if (!ctrl.Connect(lanxin_ip, lanxin_port))
    {
        return 1;
    }

    if (!ctrl.SetTopoEndpoint(lanxin_ip, lanxin_port))
    {
        std::cerr << "[chassis] RPC 连接失败 " << lanxin_ip << ":" << lanxin_port << std::endl;
        return 1;
    }

    cancelCharging(ctrl);
    move_topo_until_ok(ctrl, "fang1", 0);
    move_topo_until_ok(ctrl, "fang1", 0);
    ctrl.RotateInPlaceRetry(-90, 10, 2);

    return 0;

    ctrl.RotateInPlaceRetry(90, 10, 2);

    {
        Eigen::Matrix<double, 1, 6> posup_down_3_layer;
        posup_down_3_layer << 0.012502, 0.0, 0.642464, -1.5708, -1.5703, 3.14159;

        init_socketcan();
        hand_socketid_bind();
        WaistRobot Ti5_waist;
        Ti5_waist.moveLToPos(posup_down_3_layer, 0.1);
        return 0;
    }

    move_topo_until_ok(ctrl, "fang1", 0);
    return 0;
    chargeInMobileZone(ctrl);

    while (1)
    {
        double num = getBatteryLevel(ctrl);
        if (num > 0.9)
        {
            cancelCharging(ctrl);
            break;
        }
        else
        {
            continue;
        }
    }

    return 0;
}


//备份
int mdfdfain()
{

    WaistRobot Ti5_waist;

    init_socketcan();
    hand_socketid_bind();

    {
        int waist_p[3] = {0, 40 * 65536 * 4 / 360, 0};
        uint32_t waist_canID[] = {1};
        socketcan_sendcommand(waist_id, 1, waist_canID, 30, waist_p);
    }

    Eigen::Matrix<double, 1, 6> pos = Ti5_waist.getTcpPos();
    pos(0) = 0;
    Ti5_waist.moveLToPos(pos, 0.1);
    sleep(1);

    while (1)
    {
        double a;
        cin >> a;

        pos(0) += a;
        Ti5_waist.moveLToPos(pos, 0.1);
        sleep(1);
    }

    int a;
    cin >> a;

    pos(0) = 0.1;
    Ti5_waist.moveLToPos(pos, 0.1);
    sleep(1);

    cin >> a;
    pos(0) = 0.2;
    Ti5_waist.moveLToPos(pos, 0.1);
    sleep(1);

    cin >> a;
    pos(0) = 0.3;
    Ti5_waist.moveLToPos(pos, 0.1);
    sleep(1);
    cin >> a;
    pos(0) = -0.1;
    Ti5_waist.moveLToPos(pos, 0.1);
    sleep(1);
    cin >> a;
    pos(0) = -0.2;
    Ti5_waist.moveLToPos(pos, 0.1);
    sleep(1);
    cin >> a;
    pos(0) = -0.3;
    Ti5_waist.moveLToPos(pos, 0.1);
    sleep(1);

    return 0;
}

int mfdfdfdfdfdfain()
{
    install_app_sigint_handler();
    init_socketcan();
    hand_socketid_bind();
    rev_motor_error(0);
    rev_motor_error(1);
    motor_speed_change();
    sleep(1);

    const std::string arm_yaml = project_root_dir() + "/config/Robot_Arm_Model.yaml";
    Robot_Arm Taihu_r("T7", "T170", "right", arm_yaml);
    Robot_Arm Taihu_l("T7", "T170", "left", arm_yaml);
    start_pos(Taihu_r, Taihu_l);

    Eigen::Matrix<double, 1, 6> goal;
    goal << 0.545583, -0.010339, -0.245000, -1.570796, 0.785398, 0.000000;

    cout << "[test] 目标( place_pose right cmd )=" << goal << endl;
    const int ret = arm_line_move(Taihu_r, goal, 0.2);
    cout << "[test] arm_line_move ret=" << ret << endl;
    if (ret != 0)
    {
        cerr << "[test] 轨迹解析失败 ret=" << ret << endl;
        return 1;
    }

    Eigen::Matrix<double, 1, 6> tcp_after = arm_get_tcp_pos(Taihu_r);
    cout << "[test] 运动完成后 TCP=" << tcp_after << endl;

    sleep(3);

    Eigen::Matrix<double, 1, 6> tcp_after_sleep = arm_get_tcp_pos(Taihu_r);
    cout << "[test] 休眠3s后 TCP=" << tcp_after_sleep << endl;

    return 0;
}
int main23() // 第六排弯腰抓取验证
{
    install_app_sigint_handler();

    std::string cfg_err;
    if (!load_move_box_config(default_move_box_config_path(), g_move_cfg, cfg_err))
    {
        std::cerr << "[cfg] 加载失败，使用内置默认值: " << cfg_err << std::endl;
        g_move_cfg = default_move_box_config();
    }

    LanxinControl ctrl;
    const std::string lanxin_ip = "192.168.100.201";
    const int lanxin_port = 9000;
    if (!ctrl.Connect(lanxin_ip, lanxin_port))
        return 1;
    if (!ctrl.SetTopoEndpoint(lanxin_ip, lanxin_port))
    {
        std::cerr << "[chassis] RPC 连接失败\n";
        return 1;
    }
    move_topo_until_ok(ctrl, "c01", 0);
    row6_wait_stage("阶段1: 底盘已到 c01");

    init_socketcan();
    hand_socketid_bind();
    rev_motor_error(0);
    rev_motor_error(1);
    motor_speed_change();
    sleep(1);

    {
        double waist_angles[1] = {0};
        uint32_t waist_canID[] = {1};
        smooth_motor_move_deg(waist_id, 1, waist_canID, waist_angles, 30.0);
    }
    row6_wait_stage("阶段2: 腰1号已回0°");

    gripper::Config gcfg;
    gcfg.grasp_torque_limit_nm = 4.5;
    gcfg.soft_torque_limit_nm = 4.0;
    gripper::Gripper g(gcfg);
    if (!g.start())
    {
        std::cerr << "[row6] 夹爪启动失败\n";
        return 1;
    }

    WaistRobot Ti5_waist;
    Eigen::Matrix<double, 1, 6> pos_layer3 = g_move_cfg.waist.layer3_home;
    Ti5_waist.moveLToPos(pos_layer3, 0.1);
    sleep(g_move_cfg.waist.move_settle_sec);

    const std::string arm_yaml = project_root_dir() + "/config/Robot_Arm_Model.yaml";
    Robot_Arm Taihu_r("T7", "T170", "right", arm_yaml);
    Robot_Arm Taihu_l("T7", "T170", "left", arm_yaml);

    cout << "[row6] 双臂到待机位 (yaml standby)\n";
    start_pos(Taihu_r, Taihu_l);
    if (!wait_arms_motors_stopped(true, true, 15.0, 6, "待机"))
    {
        std::cerr << "[row6] 待机停稳失败\n";
        return 1;
    }
    row6_wait_stage("阶段3: 腰 layer3 home + 双臂待机");

    if (!run_row6_place_experiment(g, Taihu_r, Taihu_l, Ti5_waist, pos_layer3))
    {
        std::cerr << "[row6] 实验流程失败\n";
        g.stop();
        return 1;
    }

    g.stop();
    return 0;
}

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
        if (!load_cam2robot_matrix(default_camera_to_robot_yaml_path(), cam2robot, err)) // 读头相机→基座标定矩阵
        {
            std::cerr << "读取 cam2robot 失败: " << err << std::endl;
        }
        else // 头相机标定加载成功，进入本轮抓取流程
        {

            log_phase_banner("抓取流程：头相机识别分配(class0)");
            g.openAndWait(gripper::Side::Right);
            g.openAndWait(gripper::Side::Left);

            HeadAssignState st =
                detect_and_assign_head(cameras, bridge, cam2robot, true); // 头相机拍一次；任一侧未分配则再拍一次

            cout << "[head] goal_last_r: " << st.goal_last_r << endl;
            cout << "[head] goal_last_l: " << st.goal_last_l << endl;

            // 头部分配结果检查：未识别/超范围则跳过该侧；双手都无效则整轮跳过
            if (!st.have_r)
                cout << "[head] 未识别到右手，跳过右手手相机/抓取\n";
            if (!st.have_l)
                cout << "[head] 未识别到左手，跳过左手手相机/抓取\n";
            if (!st.move_r && st.have_r)
                log_arm_skip_right();
            if (!st.move_l && st.have_l)
                log_arm_skip_left();
            if (!st.move_r && !st.move_l)
                continue;

            // 头部分配且 xy/z 合格的手才参与手相机/抓取；stagger 表示该侧 x 太远需分侧或腰前进
            const bool enable_r_hand = st.move_r;
            const bool enable_l_hand = st.move_l;

            bool stagger_r = st.move_r && head_x_needs_stagger(st.goal_last_r(0));
            bool stagger_l = st.move_l && head_x_needs_stagger(st.goal_last_l(0));

            bool waist_adjusted = false; // 本轮是否动过腰，结束时要 restore
            GraspStaggerWaistState grasp_stagger;

            auto refresh_stagger_flags = [&]()
            {
                const int max_steps = std::max(1, g_move_cfg.waist.stagger_max_steps);
                if (grasp_stagger.steps_used >= max_steps)
                {
                    stagger_r = false;
                    stagger_l = false;
                    return;
                }
                stagger_r = st.move_r && head_x_needs_stagger(st.goal_last_r(0));
                stagger_l = st.move_l && head_x_needs_stagger(st.goal_last_l(0));
            };

            bool grasped_r = false;
            bool grasped_l = false;
            bool lifted_r = false;
            bool lifted_l = false;

            auto finish_grasp_and_place = [&]() -> bool
            {
                if (!grasped_r && !grasped_l)
                {
                    cout << "[arm] 本轮无抓取，跳过放货\n";
                    if (waist_adjusted)
                        restore_waist_layer3_x(Ti5_waist, posup_down_3_layer);
                    waist_adjusted = false;
                    return false;
                }

                log_phase_banner("抓取完成 → 放货流程");

                lift_grasped_after_pick(
                    Taihu_r, Taihu_l, cameras, st, grasped_r, grasped_l, lifted_r, lifted_l);
                if (grasped_r)
                    lifted_r = true;
                if (grasped_l)
                    lifted_l = true;

                move_arms_to_standby(Taihu_r, Taihu_l);

                const bool place_ok = run_post_grasp_place_flow(
                    g,
                    Taihu_r,
                    Taihu_l,
                    st,
                    grasped_r,
                    grasped_l,
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
                return place_ok;
            };

            if (stagger_r && !stagger_l && st.move_l) // 右手 x 过远：先抓左手→pos→腰进→再抓右手
            {
                cout << "[stagger] 右手 x>" << g_move_cfg.stagger.head_x_threshold
                     << "，先执行左手\n";
                const ArmLineMoveResult head_tr_l =
                    run_head_approach_selective(Taihu_r, Taihu_l, st, false, true);
                HandMoveState hs_l = run_hand_detect_and_validate(
                    Taihu_r, Taihu_l, cameras, bridge, st, false, enable_l_hand, err);
                if (on_hand_steady_timeout_abort(hs_l, false, enable_l_hand, Taihu_r, Taihu_l))
                    continue;
                if (!hs_l.move_l || head_tr_l.ret_l != 0)
                {
                    log_arm_skip_left();
                    move_arms_to_standby(Taihu_r, Taihu_l);
                    continue;
                }
                const ArmLineMoveResult grasp_tr_l =
                    run_hand_approach_and_grasp(g, Taihu_r, Taihu_l, st, hs_l);
                if (grasp_tr_l.ret_l == 0)
                {
                    grasped_l = true;
                    lift_grasped_selective(Taihu_r, Taihu_l, cameras, st, false, true);
                    lifted_l = true;
                }
                move_arms_to_standby(Taihu_r, Taihu_l);

                cout << "[stagger] 腰前进后抓右手(本轮腰进预算 "
                     << g_move_cfg.waist.stagger_max_steps << " 步)\n";
                waist_adjusted = true;
                if (!advance_waist_stagger_side(
                        Ti5_waist, posup_down_3_layer, true, st, grasp_stagger))
                {
                    cout << "[stagger] 腰前进后右手仍不可用，已抓左手，转部分放货\n";
                    restore_waist_layer3_x(Ti5_waist, posup_down_3_layer);
                    waist_adjusted = false;
                    finish_grasp_and_place();
                    continue;
                }
                refresh_stagger_flags();

                if (!st.move_r)
                {
                    log_arm_skip_right();
                    restore_waist_layer3_x(Ti5_waist, posup_down_3_layer);
                    waist_adjusted = false;
                    finish_grasp_and_place();
                    continue;
                }

                cout << "[stagger] 执行右手（左手已抓，goal_last_l 已保留）\n";
                const ArmLineMoveResult head_tr_r =
                    run_head_approach_selective(Taihu_r, Taihu_l, st, true, false);
                HandMoveState hs_r = run_hand_detect_and_validate(
                    Taihu_r, Taihu_l, cameras, bridge, st, true, false, err);
                (void)on_hand_steady_timeout_abort(hs_r, true, false, Taihu_r, Taihu_l);
                if (!hs_r.steady_timed_out && hs_r.move_r && head_tr_r.ret_r == 0)
                {
                    const ArmLineMoveResult grasp_tr_r =
                        run_hand_approach_and_grasp(g, Taihu_r, Taihu_l, st, hs_r);
                    if (grasp_tr_r.ret_r == 0)
                    {
                        grasped_r = true;
                        lift_grasped_selective(Taihu_r, Taihu_l, cameras, st, true, false);
                        lifted_r = true;
                    }
                    move_arms_to_standby(Taihu_r, Taihu_l);
                }
                else if (!hs_r.steady_timed_out)
                {
                    if (head_tr_r.ret_r != 0 || !hs_r.move_r)
                        log_arm_skip_right();
                }
                finish_grasp_and_place();
                continue;
            }

            if (stagger_l && !stagger_r && st.move_r) // 左手 x 过远：先抓右手→pos→腰进→再抓左手
            {
                cout << "[stagger] 左手 x>" << g_move_cfg.stagger.head_x_threshold
                     << "，先执行右手\n";
                const ArmLineMoveResult head_tr_r =
                    run_head_approach_selective(Taihu_r, Taihu_l, st, true, false);
                HandMoveState hs_r = run_hand_detect_and_validate(
                    Taihu_r, Taihu_l, cameras, bridge, st, enable_r_hand, false, err);
                if (on_hand_steady_timeout_abort(hs_r, enable_r_hand, false, Taihu_r, Taihu_l))
                    continue;
                if (!hs_r.move_r || head_tr_r.ret_r != 0)
                {
                    log_arm_skip_right();
                    move_arms_to_standby(Taihu_r, Taihu_l);
                    continue;
                }
                const ArmLineMoveResult grasp_tr_r =
                    run_hand_approach_and_grasp(g, Taihu_r, Taihu_l, st, hs_r);
                if (grasp_tr_r.ret_r != 0)
                {
                    move_arms_to_standby(Taihu_r, Taihu_l);
                    continue;
                }
                grasped_r = true;
                lift_grasped_selective(Taihu_r, Taihu_l, cameras, st, true, false);
                lifted_r = true;
                move_arms_to_standby(Taihu_r, Taihu_l);

                cout << "[stagger] 腰前进后抓左手(本轮腰进预算 "
                     << g_move_cfg.waist.stagger_max_steps << " 步)\n";
                waist_adjusted = true;
                if (!advance_waist_stagger_side(
                        Ti5_waist, posup_down_3_layer, false, st, grasp_stagger))
                {
                    cout << "[stagger] 腰前进后左手仍不可用，已抓右手，转部分放货\n";
                    restore_waist_layer3_x(Ti5_waist, posup_down_3_layer);
                    waist_adjusted = false;
                    finish_grasp_and_place();
                    continue;
                }
                refresh_stagger_flags();

                if (!st.move_l)
                {
                    log_arm_skip_left();
                    restore_waist_layer3_x(Ti5_waist, posup_down_3_layer);
                    waist_adjusted = false;
                    finish_grasp_and_place();
                    continue;
                }

                cout << "[stagger] 执行左手（右手已抓，goal_last_r 已保留）\n";
                const ArmLineMoveResult head_tr_l =
                    run_head_approach_selective(Taihu_r, Taihu_l, st, false, true);
                HandMoveState hs_l = run_hand_detect_and_validate(
                    Taihu_r, Taihu_l, cameras, bridge, st, false, true, err);
                (void)on_hand_steady_timeout_abort(hs_l, false, true, Taihu_r, Taihu_l);
                if (!hs_l.steady_timed_out && hs_l.move_l && head_tr_l.ret_l == 0)
                {
                    const ArmLineMoveResult grasp_tr_l =
                        run_hand_approach_and_grasp(g, Taihu_r, Taihu_l, st, hs_l);
                    if (grasp_tr_l.ret_l == 0)
                    {
                        grasped_l = true;
                        lift_grasped_selective(Taihu_r, Taihu_l, cameras, st, false, true);
                        lifted_l = true;
                    }
                    move_arms_to_standby(Taihu_r, Taihu_l);
                }
                else if (!hs_l.steady_timed_out)
                {
                    if (head_tr_l.ret_l != 0 || !hs_l.move_l)
                        log_arm_skip_left();
                }
                finish_grasp_and_place();
                continue;
            }

            if (stagger_r && stagger_l) // 双手 x 都过远：腰进最多 stagger_max_steps 次，再抓
            {
                cout << "[stagger] 双手 x>" << g_move_cfg.stagger.head_x_threshold << "，腰前进\n";
                waist_adjusted = true;
                if (!advance_waist_stagger_dual_hands(
                        Ti5_waist, posup_down_3_layer, st, grasp_stagger))
                {
                    restore_waist_layer3_x(Ti5_waist, posup_down_3_layer);
                    waist_adjusted = false;
                    continue;
                }
                refresh_stagger_flags();
            }

            if (stagger_r && !stagger_l && !st.move_l && st.move_r) // 仅右手有效且 x 过远
            {
                cout << "[stagger] 仅右手有效且 x>" << g_move_cfg.stagger.head_x_threshold
                     << "，腰前进后执行右手\n";
                waist_adjusted = true;
                if (!advance_waist_stagger_side(
                        Ti5_waist, posup_down_3_layer, true, st, grasp_stagger))
                {
                    restore_waist_layer3_x(Ti5_waist, posup_down_3_layer);
                    waist_adjusted = false;
                    continue;
                }
                refresh_stagger_flags();
                st.move_l = false;
            }
            else if (stagger_l && !stagger_r && !st.move_r && st.move_l) // 仅左手有效且 x 过远
            {
                cout << "[stagger] 仅左手有效且 x>" << g_move_cfg.stagger.head_x_threshold
                     << "，腰前进后执行左手\n";
                waist_adjusted = true;
                if (!advance_waist_stagger_side(
                        Ti5_waist, posup_down_3_layer, false, st, grasp_stagger))
                {
                    restore_waist_layer3_x(Ti5_waist, posup_down_3_layer);
                    waist_adjusted = false;
                    continue;
                }
                refresh_stagger_flags();
                st.move_r = false;
            }

            // 正常双臂流程：侧区双手同时抓；含中间区或仅单侧区则右手先抓回待机再左手
            if (head_grasp_simultaneous(st))
            {
                cout << "[head] 左右侧区均有目标，双手同时抓取\n";
                const ArmLineMoveResult head_tr =
                    run_head_approach_selective(Taihu_r, Taihu_l, st, st.move_r, st.move_l);

                HandMoveState hs = run_hand_detect_and_validate(
                    Taihu_r,
                    Taihu_l,
                    cameras,
                    bridge,
                    st,
                    enable_r_hand,
                    enable_l_hand,
                    err);

                if (on_hand_steady_timeout_abort(hs, enable_r_hand, enable_l_hand, Taihu_r, Taihu_l))
                {
                    if (waist_adjusted)
                        restore_waist_layer3_x(Ti5_waist, posup_down_3_layer);
                    waist_adjusted = false;
                    continue;
                }

                hs.move_r = hs.move_r && enable_r_hand && (!st.move_r || head_tr.ret_r == 0);
                hs.move_l = hs.move_l && enable_l_hand && (!st.move_l || head_tr.ret_l == 0);
                if (!hs.move_r && enable_r_hand)
                    log_arm_skip_right();
                if (!hs.move_l && enable_l_hand)
                    log_arm_skip_left();
                if (!hs.move_r && !hs.move_l)
                {
                    if (waist_adjusted)
                        restore_waist_layer3_x(Ti5_waist, posup_down_3_layer);
                    waist_adjusted = false;
                    continue;
                }

                const ArmLineMoveResult grasp_tr =
                    run_hand_approach_and_grasp(g, Taihu_r, Taihu_l, st, hs);
                grasped_r = hs.move_r && grasp_tr.ret_r == 0;
                grasped_l = hs.move_l && grasp_tr.ret_l == 0;
            }
            else
            {
                cout << "[head] 含中间区或侧区不全，依次：右手抓取→待机→左手抓取→待机\n";

                if (enable_r_hand && st.move_r)
                {
                    const ArmLineMoveResult head_tr_r =
                        run_head_approach_selective(Taihu_r, Taihu_l, st, true, false);
                    HandMoveState hs_r = run_hand_detect_and_validate(
                        Taihu_r, Taihu_l, cameras, bridge, st, true, false, err);
                    (void)on_hand_steady_timeout_abort(hs_r, true, false, Taihu_r, Taihu_l);
                    if (!hs_r.steady_timed_out && hs_r.move_r && head_tr_r.ret_r == 0)
                    {
                        const ArmLineMoveResult grasp_tr_r =
                            run_hand_approach_and_grasp(g, Taihu_r, Taihu_l, st, hs_r);
                        if (grasp_tr_r.ret_r == 0)
                        {
                            grasped_r = true;
                            lift_grasped_selective(Taihu_r, Taihu_l, cameras, st, true, false);
                            lifted_r = true;
                        }
                        arm_dual_line_move(Taihu_r, posr, Taihu_l, posl, 0.2);
                        sleep(1);
                    }
                    else if (!hs_r.steady_timed_out)
                    {
                        if (head_tr_r.ret_r != 0 || !hs_r.move_r)
                            log_arm_skip_right();
                    }
                }

                if (enable_l_hand && st.move_l)
                {
                    const ArmLineMoveResult head_tr_l =
                        run_head_approach_selective(Taihu_r, Taihu_l, st, false, true);
                    HandMoveState hs_l = run_hand_detect_and_validate(
                        Taihu_r, Taihu_l, cameras, bridge, st, false, true, err);
                    (void)on_hand_steady_timeout_abort(hs_l, false, true, Taihu_r, Taihu_l);
                    if (!hs_l.steady_timed_out && hs_l.move_l && head_tr_l.ret_l == 0)
                    {
                        const ArmLineMoveResult grasp_tr_l =
                            run_hand_approach_and_grasp(g, Taihu_r, Taihu_l, st, hs_l);
                        if (grasp_tr_l.ret_l == 0)
                        {
                            grasped_l = true;
                            lift_grasped_selective(Taihu_r, Taihu_l, cameras, st, false, true);
                            lifted_l = true;
                        }
                        arm_dual_line_move(Taihu_r, posr, Taihu_l, posl, 0.2);
                        sleep(1);
                    }
                    else if (!hs_l.steady_timed_out)
                    {
                        if (head_tr_l.ret_l != 0 || !hs_l.move_l)
                            log_arm_skip_left();
                    }
                }

                if (!grasped_r && !grasped_l)
                {
                    if (waist_adjusted)
                        restore_waist_layer3_x(Ti5_waist, posup_down_3_layer);
                    waist_adjusted = false;
                    continue;
                }
            }

            finish_grasp_and_place();
            continue;
        }

        sleep(2); // 每轮间隔
    }

    std::cout << "\n已退出\n";
    g.stop();
    pipeline.shutdown(kDebugVisualize);
    return 0;
}
