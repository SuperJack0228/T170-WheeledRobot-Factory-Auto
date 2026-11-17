#include "t170/app/maintenance.h"
#include "../../tasks/internal/runtime.h"
#include "t170/skills/conveyor.h"

#include "t170/telemetry/factory_status.h"
#include "t170/telemetry/dispatch_metrics.h"
#include "Ti5_Arm.h"
#include "t170/common/units.h"
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
int vision_grasp_once()
{
    return vision_grasp_with_engine(SegEngineId::Default, false, false, false);
}

int vision_servo_dual_once()
{
    if(!g_move_cfg.cameras.hand_descent_correct) {
        std::cerr << "[servo] 双手验证要求 cameras.hand_descent_correct=true\n";
        return -30;
    }
    struct Restore {
        bool saved=g_move_cfg.visual_servo.require_dual;
        ~Restore() { g_move_cfg.visual_servo.require_dual=saved; }
    } restore;
    g_move_cfg.visual_servo.require_dual=true;
    return vision_grasp_with_engine(SegEngineId::Default,false,false,false);
}






}
