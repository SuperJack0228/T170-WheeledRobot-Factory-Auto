#include "t170/common/paths.h"
#include "t170/common/logging.h"
#include "t170/telemetry/factory_status.h"
#include "t170/telemetry/dispatch_metrics.h"

#include "t170/common/abort_query.h"

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>

#include "internal/console.h"
namespace orch {
namespace {
int cloexec_pipe(int fds[2]) {
#ifdef __linux__
    return ::pipe2(fds,O_CLOEXEC);
#else
    if(::pipe(fds)!=0)return -1;
    if(::fcntl(fds[0],F_SETFD,FD_CLOEXEC)==-1 || ::fcntl(fds[1],F_SETFD,FD_CLOEXEC)==-1) {
        const int error=errno;::close(fds[0]);::close(fds[1]);errno=error;return -1;
    }return 0;
#endif
}
}

namespace {
constexpr size_t kKeepLogs = 30;
}

namespace {
std::string now_stamp()
{
    return t170::logging::local_time("%Y%m%d_%H%M%S");
}
}

namespace {
struct Console
{
    std::mutex mu;
    bool on = false;
    std::atomic<bool> stopping{false};
    int saved_out = -1;
    int saved_err = -1;
    int rd_out = -1;
    int rd_err = -1;
    std::thread th_out;
    std::thread th_err;
    std::ofstream file;
};
}

namespace {
Console g_con;
}

namespace {
void write_fd(int fd, const std::string &s)
{
    const char *p = s.data();
    size_t left = s.size();
    while (left > 0)
    {
        const ssize_t n = ::write(fd, p, left);
        if (n < 0)
        {
            if (errno == EINTR)
                continue;
            return;
        }
        p += n;
        left -= static_cast<size_t>(n);
    }
}
}

namespace {
void handle_raw_line(const std::string &line, bool is_err)
{
    std::lock_guard<std::mutex> lk(g_con.mu);
    const std::string t = now_hms();
    if (g_con.file)
    {
        g_con.file << t << (is_err ? " E " : "   ") << line << '\n';
        g_con.file.flush();
    }
    if (is_err && g_con.saved_err >= 0 && !line.empty())
        write_fd(g_con.saved_err, t + (line.find("警告") == std::string::npos ? "  警告  " : "  ") +
                                      line + "\n");
}
}

namespace {
void pump(int rd, bool is_err)
{
    std::string buf;
    char tmp[4096];
    while (true)
    {
        pollfd p{rd, POLLIN, 0};
        const int pr = ::poll(&p, 1, 200);
        if (pr < 0 && errno == EINTR)
            continue;
        if (pr <= 0)
        {
            if (g_con.stopping.load())
                break;
            continue;
        }
        const ssize_t n = ::read(rd, tmp, sizeof(tmp));
        if (n < 0 && errno == EINTR)
            continue;
        if (n <= 0)
            break;
        buf.append(tmp, static_cast<size_t>(n));
        size_t pos = 0;
        while (true)
        {
            const size_t nl = buf.find('\n', pos);
            if (nl == std::string::npos)
                break;
            std::string line = buf.substr(pos, nl - pos);
            if (!line.empty() && line.back() == '\r')
                line.pop_back();
            handle_raw_line(line, is_err);
            pos = nl + 1;
        }
        buf.erase(0, pos);
    }
    if (!buf.empty())
        handle_raw_line(buf, is_err);
}
}

namespace {
void prune_logs(const std::filesystem::path &dir)
{
    std::error_code ec;
    std::vector<std::filesystem::path> logs;
    for (const auto &e : std::filesystem::directory_iterator(dir, ec))
    {
        const std::string name = e.path().filename().string();
        if (name.rfind("dispatch_", 0) == 0 && e.path().extension() == ".log")
            logs.push_back(e.path());
    }
    if (logs.size() <= kKeepLogs)
        return;
    std::sort(logs.begin(), logs.end());
    for (size_t i = 0; i + kKeepLogs < logs.size(); ++i)
        std::filesystem::remove(logs[i], ec);
}
}

std::string console_start()
{
    if (const char *v = std::getenv("T170_DISPATCH_VERBOSE"))
    {
        if (v[0] != '\0' && std::strcmp(v, "0") != 0)
            return {};
    }
    const std::filesystem::path dir = t170::paths::log_directory();
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    prune_logs(dir);
    const std::string path = (dir / ("dispatch_" + now_stamp() + ".log")).string();

    std::lock_guard<std::mutex> lk(g_con.mu);
    if (g_con.on)
        return {};
    g_con.file.open(path, std::ios::out | std::ios::app);
    if (!g_con.file)
        return {};

    int po[2] = {-1, -1};
    int pe[2] = {-1, -1};
    if (cloexec_pipe(po) != 0)
        return {};
    if (cloexec_pipe(pe) != 0)
    {
        ::close(po[0]);
        ::close(po[1]);
        return {};
    }
    std::cout.flush();
    std::cerr.flush();
    std::fflush(stdout);
    std::fflush(stderr);
    g_con.saved_out = ::fcntl(1, F_DUPFD_CLOEXEC, 3);
    g_con.saved_err = ::fcntl(2, F_DUPFD_CLOEXEC, 3);
    ::dup2(po[1], 1);
    ::dup2(pe[1], 2);
    ::close(po[1]);
    ::close(pe[1]);
    g_con.rd_out = po[0];
    g_con.rd_err = pe[0];
    g_con.stopping = false;
    g_con.th_out = std::thread(pump, g_con.rd_out, false);
    g_con.th_err = std::thread(pump, g_con.rd_err, true);
    g_con.on = true;
    return path;
}
void console_stop()
{
    {
        std::lock_guard<std::mutex> lk(g_con.mu);
        if (!g_con.on)
            return;
    }
    std::cout.flush();
    std::cerr.flush();
    std::fflush(stdout);
    std::fflush(stderr);
    ::dup2(g_con.saved_out, 1);
    ::dup2(g_con.saved_err, 2);
    g_con.stopping = true;
    if (g_con.th_out.joinable())
        g_con.th_out.join();
    if (g_con.th_err.joinable())
        g_con.th_err.join();
    std::lock_guard<std::mutex> lk(g_con.mu);
    ::close(g_con.rd_out);
    ::close(g_con.rd_err);
    ::close(g_con.saved_out);
    ::close(g_con.saved_err);
    g_con.rd_out = g_con.rd_err = g_con.saved_out = g_con.saved_err = -1;
    g_con.file.close();
    g_con.on = false;
}
void print_event(const std::string &line)
{
    {
        std::lock_guard<std::mutex> lk(g_con.mu);
        if (g_con.on)
        {
            if (g_con.file)
            {
                g_con.file << "== " << line << '\n';
                g_con.file.flush();
            }
            write_fd(g_con.saved_out, line + "\n");
            return;
        }
    }
    std::cout << "[调度面板] " << line << std::endl;
}
}
