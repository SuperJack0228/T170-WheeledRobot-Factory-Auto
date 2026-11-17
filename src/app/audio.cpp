#include "t170/common/announcements.h"
#include "t170/common/paths.h"
#include <cerrno>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

namespace orch {
namespace {
std::mutex g_announce_mu;
}

namespace {
pid_t g_announce_pid = -1;
}

namespace {
bool g_failure_announced = false;
}

namespace {
void stop_announce_locked()
{
    if (g_announce_pid <= 0)
        return;
    kill(-g_announce_pid, SIGTERM);
    int status = 0;
    while (waitpid(g_announce_pid, &status, 0) < 0 && errno == EINTR)
    {
    }
    g_announce_pid = -1;
}
}

bool play_boot_clip(const std::string &name, std::string &err, bool once_if_failure)
{
    err.clear();
    std::lock_guard<std::mutex> lock(g_announce_mu);
    if (once_if_failure && name == "failure" && g_failure_announced)
        return true;
    stop_announce_locked();
    const std::string path = t170::paths::resource("assets/audio/" + name + ".mp3").string();
    if (!std::filesystem::exists(path))
    {
        err = "找不到 " + path;
        return false;
    }
    // 路径在 fork 前拼好。子进程只做 exec，避免别的线程正占着分配锁时在子进程里再分配。
    const std::string location = "location=" + path;
    const pid_t pid = fork();
    if (pid < 0)
    {
        err = "无法启动播放";
        return false;
    }
    if (pid == 0)
    {
        setpgid(0, 0);
        execl("/usr/bin/gst-launch-1.0", "gst-launch-1.0", "-q", "filesrc", location.c_str(), "!",
              "decodebin", "!", "audioconvert", "!", "audioresample", "!", "alsasink",
              "device=plughw:CARD=Device,DEV=0", static_cast<char *>(nullptr));
        _exit(127);
    }
    setpgid(pid, pid);
    g_announce_pid = pid;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(45);
    int status = 0;
    bool timed_out = false;
    while (true)
    {
        const pid_t got = waitpid(pid, &status, WNOHANG);
        if (got == pid)
            break;
        if (got < 0 && errno != EINTR)
        {
            err = "等待播放结束失败";
            break;
        }
        if (std::chrono::steady_clock::now() >= deadline)
        {
            kill(-pid, SIGKILL);
            kill(pid, SIGKILL);
            while (waitpid(pid, &status, 0) < 0 && errno == EINTR)
            {
            }
            err = "播放超过 45 秒，已停";
            timed_out = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (g_announce_pid == pid)
        g_announce_pid = -1;
    if (timed_out || !err.empty())
        return false;
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
    {
        err = "播放器退出 code=" + std::to_string(WIFEXITED(status) ? WEXITSTATUS(status) : status);
        return false;
    }
    if (once_if_failure && name == "failure")
        g_failure_announced = true;
    std::cout << "[announce] " << name << "\n" << std::flush;
    return true;
}

void boot_announce(const std::string &name)
{
    std::string err;
    if (!play_boot_clip(name, err, true) && !err.empty())
        std::cerr << "[announce] " << name << " 播放失败: " << err << "\n";
}
}
