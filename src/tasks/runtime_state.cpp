#include "internal/runtime.h"
#include "t170/telemetry/dispatch_metrics.h"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace orch {
HwSession *g_hw = nullptr;

int g_load_parts_seen = -1;

bool g_grasp_departed = false;

std::function<void()> g_on_grasp_depart;

bool g_unload_empty_known = false;

bool g_dispatch_stay_at_tray2 = false;

std::string g_photo_ready_station;

bool g_tray2_place_prep_done = false;

int g_unload_empty_right = 0;

int g_unload_empty_left = 0;

std::string g_dispatch_nav_fault;


std::chrono::steady_clock::time_point g_station_arrived_at{};

bool g_station_arrived = false;

std::thread g_belt_depart_th;

std::atomic<int> g_belt_depart_rc{0};

bool g_belt_depart_started = false;

std::thread g_tray2_waist_th;

std::atomic<int> g_tray2_waist_rc{0};

bool g_tray2_waist_started = false;

bool g_body_disabled_for_charge = false;
}
