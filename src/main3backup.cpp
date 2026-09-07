
int mafdfdfdfin()
{
    gripper::Config cfg;
    cfg.grasp_torque_limit_nm = 2.0;
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

    g.openAndWait(gripper::Side::Left);
    g.closeAndWait(gripper::Side::Left);
    g.openAndWait(gripper::Side::Right);
    g.closeAndWait(gripper::Side::Right);

    g.openAndWait(gripper::Side::Left);
    g.openAndWait(gripper::Side::Right);

    std::this_thread::sleep_for(std::chrono::seconds(2));

    g.openSoftAndWait(gripper::Side::Left);
    g.closeSoftAndWait(gripper::Side::Left);
    g.openSoftAndWait(gripper::Side::Right);
    g.closeSoftAndWait(gripper::Side::Right);

    g.openSoftAndWait(gripper::Side::Left);
    g.openSoftAndWait(gripper::Side::Right);

    g.stop();
    return 0;
}

int mdfdfdfdain()
{
    PosePipeline pipeline;
    std::string err;
    if (!pipeline.init(err))
    {
        std::cerr << "初始化失败: " << err << std::endl;
        return 1;
    }

    RealSenseMultiCam &cameras = pipeline.cameras();
    SegPoseBridge &bridge = pipeline.bridge();

    const PoseDetectionRecords head_records =
        detect_pose_at_slot(cameras, bridge, CameraSlot::Head, kDebugVisualize);
    const PoseDetectionRecords right_hand_records =
        detect_pose_at_slot(cameras, bridge, CameraSlot::RightHand, kDebugVisualize);
    const PoseDetectionRecords left_hand_records =
        detect_pose_at_slot(cameras, bridge, CameraSlot::LeftHand, kDebugVisualize);

    print_pose_records_summary("head", head_records);
    print_pose_records_summary("right_hand", right_hand_records);
    print_pose_records_summary("left_hand", left_hand_records);

    pipeline.shutdown(kDebugVisualize);
    return 0;
}
