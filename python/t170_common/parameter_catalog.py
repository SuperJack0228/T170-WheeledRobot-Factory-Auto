"""Persistent operator parameters shared by the web UI and deployment loader."""

FIELD_GROUPS = {
    "motion": [
        ("腰", [
            ("waist.ready_forward_m", "料盘抓取时，腰相对 home 再向前", "米"),
            ("waist.far_row_forward_m", "料盘一远三排，腰再向前伸", "米"),
            ("waist.ready_z_m", "抓取时腰的高度", "米"),
            ("waist.layer3_home.2", "回到 home 时腰的高度", "米"),
            ("waist.move_settle_sec", "腰到位后再停一会儿", "秒"),
            ("tray2_place.far_row_forward_m", "料盘二放置时，远排腰向前伸", "米"),
        ]),
        ("头", [
            ("head.pitch_deg", "平时低头角度", "度"),
            ("head.far_pitch_deg", "看远排时再低头的角度", "度"),
            ("head.speed_deg_s", "头转动速度", "度/秒"),
        ]),
        ("料盘高度", [
            ("tray.z_ref_m", "料盘上表面在手臂基座下的高度", "米"),
            ("head_grasp.hover_above_m", "手停在孔上方的高度", "米"),
            ("head_grasp.lift_after_grasp_z", "夹住后再抬起的高度", "米"),
            ("head_grasp.pre_grasp_settle_sec", "下到孔里后再等一会儿才合爪", "秒"),
            ("head_grasp.bezier_endpoint_xyz_tol_m", "手离目标超过这个距离就不合爪", "米"),
        ]),
        ("料盘抓取速度", [
            ("head_grasp.approach_vel_m_s", "伸向孔位的速度", "米/秒"),
            ("head_grasp.descend_vel_m_s", "下压速度", "米/秒"),
            ("head_grasp.lift_vel_m_s", "抬起速度", "米/秒"),
            ("head_grasp.return_vel_m_s", "收回速度", "米/秒"),
        ]),
        ("料盘抓取偏置，左手", [
            ("head_grasp.goal_x_offset", "前后，向前为正", "米"),
            ("head_grasp.goal_y_offset", "左右，向左为正", "米"),
            ("head_grasp.grasp_z_offset_m", "上下，向上为正", "米"),
        ]),
        ("料盘抓取偏置，右手", [
            ("head_grasp.right_goal_x_offset", "前后，向前为正", "米"),
            ("head_grasp.right_goal_y_offset", "左右，向左为正", "米"),
            ("head_grasp.right_grasp_z_offset_m", "上下，向上为正", "米"),
        ]),
        ("料盘最远一排的手腕角度，左手", [
            ("head_grasp.row1_rpy_deg.left.0", "翻滚", "度"),
            ("head_grasp.row1_rpy_deg.left.1", "俯仰", "度"),
            ("head_grasp.row1_rpy_deg.left.2", "偏航", "度"),
        ]),
        ("料盘最远一排的手腕角度，右手", [
            ("head_grasp.row1_rpy_deg.right.0", "翻滚", "度"),
            ("head_grasp.row1_rpy_deg.right.1", "俯仰", "度"),
            ("head_grasp.row1_rpy_deg.right.2", "偏航", "度"),
        ]),
        ("料盘二放置偏置，左手", [
            ("tray2_place.offset.left.x", "前后，向前为正", "米"),
            ("tray2_place.offset.left.y", "左右，向左为正", "米"),
            ("tray2_place.offset.left.z", "上下，向上为正", "米"),
        ]),
        ("料盘二放置偏置，右手", [
            ("tray2_place.offset.right.x", "前后，向前为正", "米"),
            ("tray2_place.offset.right.y", "左右，向左为正", "米"),
            ("tray2_place.offset.right.z", "上下，向上为正", "米"),
        ]),
        ("传送带一", [
            ("conveyor.waist_x", "拍照时腰向前", "米"),
            ("conveyor.waist_z", "拍照时腰的高度", "米"),
            ("conveyor.head_pitch_deg", "头低头角度", "度"),
            ("conveyor.grasp_hover_above_m", "抓取时先停在上方", "米"),
            ("conveyor.place_hover_above_m", "放置时先停在上方", "米"),
            ("conveyor.place_approach_vel_m_s", "放置伸手速度", "米/秒"),
            ("conveyor.place_retract_vel_m_s", "放置收手速度", "米/秒"),
            ("conveyor.grasp_approach_vel_m_s", "抓取伸手速度", "米/秒"),
            ("conveyor.grasp_retract_vel_m_s", "抓取收手速度", "米/秒"),
            ("conveyor.grasp_ready_close_ratio", "下去前夹爪合上的程度，0 全开 1 全合", ""),
            ("conveyor.fixed_origin_z_m", "参考机床1.2在参考腰高时的码平面基座Z；其他机床自动叠加高度差", "米"),
            ("conveyor.fixed_origin_ref_waist_z_m", "上面这个高度对应的腰高，一般保持 0.63", "米"),
            ("conveyor.photo_after_arrival_sec", "到站后放置拍照前再等", "秒"),
            ("conveyor.grasp_photo_settle_sec", "腰头到位后抓取拍照前再等", "秒"),
            ("conveyor.offset.right.x", "右手放置前后", "米"),
            ("conveyor.offset.right.y", "右手放置左右", "米"),
            ("conveyor.offset.right.z", "右手放置上下", "米"),
            ("conveyor.offset.left.x", "左手放置前后", "米"),
            ("conveyor.offset.left.y", "左手放置左右", "米"),
            ("conveyor.offset.left.z", "左手放置上下", "米"),
            ("conveyor.grasp_offset.right.x", "右手抓取前后", "米"),
            ("conveyor.grasp_offset.right.y", "右手抓取左右", "米"),
            ("conveyor.grasp_offset.right.z", "右手抓取上下", "米"),
            ("conveyor.grasp_offset.left.x", "左手抓取前后", "米"),
            ("conveyor.grasp_offset.left.y", "左手抓取左右", "米"),
            ("conveyor.grasp_offset.left.z", "左手抓取上下", "米"),
        ]),
        ("传送带二", [
            ("conveyor2.waist_x", "拍照时腰向前", "米"),
            ("conveyor2.waist_z", "拍照时腰的高度", "米"),
            ("conveyor2.head_pitch_deg", "头低头角度", "度"),
            ("conveyor2.grasp_hover_above_m", "抓取时先停在上方", "米"),
            ("conveyor2.place_hover_above_m", "放置时先停在上方", "米"),
            ("conveyor2.place_approach_vel_m_s", "放置伸手速度", "米/秒"),
            ("conveyor2.place_retract_vel_m_s", "放置收手速度", "米/秒"),
            ("conveyor2.grasp_approach_vel_m_s", "抓取伸手速度", "米/秒"),
            ("conveyor2.grasp_retract_vel_m_s", "抓取收手速度", "米/秒"),
            ("conveyor2.fixed_origin_z_m", "参考机床1.2在参考腰高时的码平面基座Z；其他机床自动叠加高度差", "米"),
            ("conveyor2.fixed_origin_ref_waist_z_m", "上面这个高度对应的腰高，一般保持 0.63", "米"),
            ("conveyor2.photo_after_arrival_sec", "到站后放置拍照前再等", "秒"),
            ("conveyor2.grasp_photo_settle_sec", "腰头到位后抓取拍照前再等", "秒"),
            ("conveyor2.offset.right.x", "右手放置前后", "米"),
            ("conveyor2.offset.right.y", "右手放置左右", "米"),
            ("conveyor2.offset.right.z", "右手放置上下", "米"),
            ("conveyor2.offset.left.x", "左手放置前后", "米"),
            ("conveyor2.offset.left.y", "左手放置左右", "米"),
            ("conveyor2.offset.left.z", "左手放置上下", "米"),
            ("conveyor2.grasp_offset.right.x", "右手抓取前后", "米"),
            ("conveyor2.grasp_offset.right.y", "右手抓取左右", "米"),
            ("conveyor2.grasp_offset.right.z", "右手抓取上下", "米"),
            ("conveyor2.grasp_offset.left.x", "左手抓取前后", "米"),
            ("conveyor2.grasp_offset.left.y", "左手抓取左右", "米"),
            ("conveyor2.grasp_offset.left.z", "左手抓取上下", "米"),
        ]),
        ("空抓检测", [
            ("gripper_check.enable", "合爪后判空抓，true 开 false 关", ""),
            ("gripper_check.empty_q_rad", "合爪结束 q 不大于这个值算空抓，要明显小于夹住零件时的 q", "弧度"),
            ("gripper_check.max_empty_retry", "同一只手空抓后最多重抓几次", "次"),
        ]),
        ("站点", [
            ("chassis.dispatch_wait_station", "调度未下发待命点时，失败/无任务回这里", ""),
            ("chassis.charge_station", "调试充电点（charge 指令；调度仍用系统下发点）", ""),
            ("chassis.charge_wait_station", "充完电后待命点", ""),
            ("chassis.charge_done_percent", "调度充电结束电量阈值", "%"),
        ]),
    ],
    "tray_code": [
        ("正在使用的四角码", [
            ("tray_grasp_code.pitch_xy_m", "四个码中心的间距", "米"),
            ("tray_grasp_code.print_size_m", "打印外边长", "米"),
            ("tray_grasp_code.black_side_m", "黑框边长", "米"),
            ("tray_grasp_code.locate_id", "左上和右下常见的码号", ""),
        ]),
        ("料盘二，和料盘一同一套四角码，不按号码分角", [
            ("tray2.print_size_m", "打印外边长", "米"),
            ("tray2.black_side_m", "黑框边长", "米"),
            ("tray2.pitch_xy_m", "四个码中心的间距", "米"),
        ]),
    ],
    "belt_code": [
        ("传送带码", [
            ("aruco.print_size_m", "打印外边长", "米"),
            ("belt.pitch_y_m", "两个码心的左右间距", "米"),
            ("belt.edge.front_m", "码心到皮带前端", "米"),
            ("belt.edge.rear_m", "码心到皮带后端", "米"),
            ("belt.edge.left_m", "码心到皮带左端", "米"),
            ("belt.edge.right_m", "码心到皮带右端", "米"),
        ]),
    ],
    "hand_eye": [
        ("相机到头部的平移，程序读的是矩阵，改这里会写回矩阵最后一列", [
            ("tx", "相机相对头部，前后方向的平移", "毫米"),
            ("ty", "相机相对头部，左右方向的平移", "毫米"),
            ("tz", "相机相对头部，上下方向的平移", "毫米"),
        ]),
    ],
    "head_geom": [
        ("头部高度", [
            ("link_length_mm", "头部电机末端离基座的高度", "毫米"),
        ]),
    ],
}


QUICK_FIELDS = frozenset(key for _, rows in FIELD_GROUPS["motion"] for key, _, _ in rows)
QUICK_VECTORS = frozenset(key.rsplit('.', 1)[0] for key in QUICK_FIELDS if key.rsplit('.', 1)[-1].isdigit())


def quick_key_allowed(key):
    parent, _, index = key.rpartition('.')
    return key in QUICK_FIELDS or (parent in QUICK_VECTORS and index.isdigit())
