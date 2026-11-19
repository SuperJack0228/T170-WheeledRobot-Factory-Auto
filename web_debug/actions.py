"""Presentation metadata; availability always comes from the C++ command catalog."""
GROUPS = [
    ("preparation", "姿态与准备", "固定关节 Home；按当前工位选择准备姿态。", [
        ("home", "双臂回 Home", "关节轨迹返回固定 Home，随后执行原有腰部归位。"),
        ("grasp_ready1", "料盘抓取 · 近排准备", "进入第一姿态组，不抓取。"),
        ("grasp_ready2", "料盘抓取 · 中排准备", "进入第二姿态组，不抓取。"),
        ("grasp_ready3", "料盘抓取 · 远排准备", "进入第三姿态组，不抓取。"),
        ("tray2ready1", "料盘放置 · 近排准备", "保持当前夹持状态，进入放置准备位。"),
        ("tray2ready2", "料盘放置 · 中排准备", "保持当前夹持状态，进入中排准备位。"),
        ("tray2ready3", "料盘放置 · 远排准备", "保持当前夹持状态，进入远排准备位。"),
        ("belt_ready", "传送带一 · 准备", "执行当前传送带一准备动作。"),
        ("belt2_ready", "传送带二 · 准备", "执行当前传送带二准备动作。"),
    ]),
    ("tray", "料盘抓放", "头相机引导与手相机闭环沿用当前技能实现。", [
        ("vision_grasp_once", "料盘抓取 · 单次", "按现有流程双手各取一件，结束保持夹持。"),
        ("tray2_auto", "料盘放置 · 当前持件", "读取左右手实际持件状态，执行空孔放置。"),
        ("vision_grasp", "料盘连续抓取测试", "持续执行现有抓取测试流程；这不是一轮生产上料任务。"),
    ]),
    ("conveyor", "传送带抓放", "完整单步工位流程可能包含底盘与腰部运动。", [
        ("belt_grasp", "传送带一 · 抓取", "数件、依次抓取、闭环修正与撤离。"),
        ("belt", "传送带一 · 放置", "执行当前传送带一放置流程。"),
        ("belt2_grasp", "传送带二 · 抓取", "数件、依次抓取、闭环修正与撤离。"),
        ("belt2", "传送带二 · 放置", "执行当前传送带二放置流程。"),
    ]),
    ("diagnostics", "识别与诊断", "识别结果在视觉页查看；标定使用独立向导。", [
        ("aruco_detect", "识别工位标记", "运行当前 ArUco 识别入口。"),
        ("detect_tray_holes", "识别料盘孔位", "运行当前料盘孔位检测入口。"),
        ("vision_servo_dual_once", "双手伺服单次诊断", "调用当前双手伺服诊断流程。"),
    ]),
    ("service", "其他维护动作", "非抓放动作集中在此，避免干扰常用操作。", [
        ("charge", "充电测试", "执行现有导航与充电测试。"),
        ("waist1", "腰部测试位一", "执行预设腰部测试动作。"),
        ("waist2", "腰部测试位二", "执行预设腰部测试动作。"),
        ("waist_cycle", "腰部循环测试", "执行当前腰部循环测试。"),
        ("wave_ready", "挥手准备", "腰部归位并使手臂下垂。"),
        ("wave", "挥手", "执行现有挥手动作。"),
        ("bow", "鞠躬", "执行现有鞠躬动作。"),
        ("welcome", "迎宾", "执行现有迎宾流程。"),
    ]),
]


def catalog(capabilities):
    commands = {entry["name"]: entry for entry in capabilities.get("commands", [])}
    groups = []
    for key, title, description, entries in GROUPS:
        actions = []
        for name, label, note in entries:
            spec = commands.get(name)
            if not spec or spec.get("group") == "retired":
                continue
            actions.append(dict(name=name, label=label, description=note, allowed=bool(spec.get("allowed"))))
        if actions:
            groups.append(dict(id=key, title=title, description=description, actions=actions))
    return {"ok": True, "mode": capabilities.get("mode"), "groups": groups}
