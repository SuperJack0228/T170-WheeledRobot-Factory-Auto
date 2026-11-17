#include "internal/state.h"
#include "t170/hardware/device_bus.h"
#include "t170/vision/debug.h"
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <thread>
#include <sys/socket.h>
#include <unistd.h>

namespace t170::app::detail {
bool read_line(int fd, std::string &out)
{
    out.clear();
    char c = 0;
    while (out.size() <= (1u << 20))
    {
        const ssize_t n = ::recv(fd, &c, 1, 0);
        if (n <= 0)
            return false;
        if (c == '\n')
            return true;
        if (c != '\r')
            out.push_back(c);
    }
    return false;
}
void write_json(int fd, const Json::Value &value)
{
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    const std::string body = Json::writeString(builder, value) + "\n";
    ::send(fd, body.data(), body.size(), MSG_NOSIGNAL);
}
void handle_client(int fd)
{
    std::string line;
    if (!read_line(fd, line))
    {
        ::close(fd);
        return;
    }
    Json::Value request;
    JSONCPP_STRING error;
    Json::CharReaderBuilder builder;
    const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    if (!reader->parse(line.data(), line.data() + line.size(), &request, &error))
        write_json(fd, reply(false, "JSON 解析失败"));
    else
    {
        try
        {
            const std::string cmd = request.get("cmd", "").asString();
            if (!command_collects_arrivals(cmd) || !command_allowed(state().mode,cmd))
                write_json(fd, dispatch(request));
            else
            {
                arm_arrival_log_clear();
                write_json(fd, with_arm_arrivals(dispatch(request)));
            }
        }
        catch (const std::exception &ex)
        {
            write_json(fd, reply(false, ex.what()));
        }
    }
    ::close(fd);
}
}
