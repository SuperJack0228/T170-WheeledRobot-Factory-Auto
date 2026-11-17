#include "t170/vision/tray_localizer.h"
#include "../internal/legacy_helpers.h"
#include "t170/motion/servo.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <numeric>
#include <optional>
#include <random>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "t170/vision/wrist_observer.h"
#include "t170/motion/arm.h"
#include "t170/common/units.h"
#include <array>
#include <cstring>
#include "t170/vision/head_transform.h"
#include "t170/hardware/device_bus.h"

using std::vector;
using std::thread;


namespace move_box {
bool g_grasp_adaptive_rpy = false;

bool g_tray1_far_band_memory = false;

double g_current_waist_layer3_z = std::numeric_limits<double>::quiet_NaN();

double g_waist_pitch_rad = 0.0;

double g_waist_roll_rad = 0.0;

double g_waist_yaw_rad = 0.0;

TrayHoleTask g_tray_hole_task = TrayHoleTask::GraspParts;

int g_assign_class_override = -1;

int g_assign_fixed_from_robot_row = 0;



bool g_waist_layer3_synced = false;
}
