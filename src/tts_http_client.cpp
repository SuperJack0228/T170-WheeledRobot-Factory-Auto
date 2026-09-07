#include "tts_http_client.h"

#include <arpa/inet.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <netdb.h>
#include <cstdlib>
#include <sstream>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <unordered_map>
#include <vector>

namespace
{
    struct ParsedUrl
    {
        std::string host;
        int port = 80;
        std::string base_path = "";
    };

    bool starts_with(const std::string &s, const std::string &prefix)
    {
        return s.rfind(prefix, 0) == 0;
    }

    std::string trim(const std::string &s)
    {
        size_t b = 0;
        while (b < s.size() && (s[b] == ' ' || s[b] == '\t' || s[b] == '\r' || s[b] == '\n'))
            ++b;
        size_t e = s.size();
        while (e > b && (s[e - 1] == ' ' || s[e - 1] == '\t' || s[e - 1] == '\r' || s[e - 1] == '\n'))
            --e;
        return s.substr(b, e - b);
    }

    std::string to_lower(std::string s)
    {
        for (char &c : s)
        {
            if (c >= 'A' && c <= 'Z')
                c = static_cast<char>(c - 'A' + 'a');
        }
        return s;
    }

    bool parse_http_base_url(const std::string &base_url, ParsedUrl &out, std::string &err)
    {
        if (!starts_with(base_url, "http://"))
        {
            err = "only http:// is supported";
            return false;
        }
        std::string rest = base_url.substr(std::strlen("http://"));
        if (rest.empty())
        {
            err = "invalid base_url";
            return false;
        }

        size_t slash = rest.find('/');
        std::string hostport = (slash == std::string::npos) ? rest : rest.substr(0, slash);
        out.base_path = (slash == std::string::npos) ? "" : rest.substr(slash);
        if (!out.base_path.empty() && out.base_path.back() == '/')
            out.base_path.pop_back();

        size_t colon = hostport.rfind(':');
        if (colon == std::string::npos)
        {
            out.host = hostport;
            out.port = 80;
        }
        else
        {
            out.host = hostport.substr(0, colon);
            std::string port_str = hostport.substr(colon + 1);
            if (port_str.empty())
            {
                err = "invalid port";
                return false;
            }
            out.port = std::stoi(port_str);
        }

        if (out.host.empty())
        {
            err = "host is empty";
            return false;
        }
        return true;
    }

    std::string json_escape(const std::string &s)
    {
        std::ostringstream oss;
        for (unsigned char c : s)
        {
            switch (c)
            {
            case '\"':
                oss << "\\\"";
                break;
            case '\\':
                oss << "\\\\";
                break;
            case '\b':
                oss << "\\b";
                break;
            case '\f':
                oss << "\\f";
                break;
            case '\n':
                oss << "\\n";
                break;
            case '\r':
                oss << "\\r";
                break;
            case '\t':
                oss << "\\t";
                break;
            default:
                if (c < 0x20)
                {
                    static const char *hex = "0123456789abcdef";
                    oss << "\\u00" << hex[(c >> 4) & 0x0F] << hex[c & 0x0F];
                }
                else
                {
                    oss << static_cast<char>(c);
                }
                break;
            }
        }
        return oss.str();
    }

    bool recv_all(int fd, std::string &out, std::string &err)
    {
        char buf[8192];
        while (true)
        {
            const ssize_t n = recv(fd, buf, sizeof(buf), 0);
            if (n > 0)
            {
                out.append(buf, static_cast<size_t>(n));
                continue;
            }
            if (n == 0)
                return true;
            if (errno == EINTR)
                continue;
            err = std::string("recv failed: ") + std::strerror(errno);
            return false;
        }
    }

    bool parse_http_response(const std::string &raw,
                             int &status,
                             std::unordered_map<std::string, std::string> &headers,
                             std::string &body,
                             std::string &err)
    {
        const std::string delim = "\r\n\r\n";
        const size_t header_end = raw.find(delim);
        if (header_end == std::string::npos)
        {
            err = "invalid HTTP response: no header terminator";
            return false;
        }

        const std::string header_block = raw.substr(0, header_end);
        body = raw.substr(header_end + delim.size());

        std::istringstream hs(header_block);
        std::string status_line;
        if (!std::getline(hs, status_line))
        {
            err = "invalid HTTP response: no status line";
            return false;
        }
        if (!status_line.empty() && status_line.back() == '\r')
            status_line.pop_back();

        {
            std::istringstream sl(status_line);
            std::string http_ver;
            sl >> http_ver >> status;
            if (http_ver.empty() || status <= 0)
            {
                err = "invalid status line: " + status_line;
                return false;
            }
        }

        std::string line;
        while (std::getline(hs, line))
        {
            if (!line.empty() && line.back() == '\r')
                line.pop_back();
            if (line.empty())
                continue;
            size_t colon = line.find(':');
            if (colon == std::string::npos)
                continue;
            std::string k = to_lower(trim(line.substr(0, colon)));
            std::string v = trim(line.substr(colon + 1));
            headers[k] = v;
        }
        return true;
    }

    std::string guess_ext(const std::string &content_type)
    {
        const std::string ct = to_lower(content_type);
        if (ct.find("wav") != std::string::npos || ct.find("wave") != std::string::npos)
            return ".wav";
        if (ct.find("mpeg") != std::string::npos || ct.find("mp3") != std::string::npos)
            return ".mp3";
        if (ct.find("ogg") != std::string::npos)
            return ".ogg";
        return ".bin";
    }

    std::string find_player_cmd()
    {
        const char *path_env = std::getenv("PATH");
        const std::string path = path_env ? path_env : "";
        std::vector<std::string> dirs;
        std::stringstream ss(path);
        std::string dir;
        while (std::getline(ss, dir, ':'))
            dirs.push_back(dir);

        const std::vector<std::string> players = {"ffplay -nodisp -autoexit", "paplay", "aplay", "mpg123"};
        for (const auto &p : players)
        {
            const std::string exe = p.substr(0, p.find(' '));
            for (const auto &d : dirs)
            {
                const std::string full = d + "/" + exe;
                if (::access(full.c_str(), X_OK) == 0)
                    return p;
            }
        }
        return "";
    }
}

namespace tts
{
    namespace
    {
        std::string make_quiet_play_command(const std::string &player, const std::string &path)
        {
            const std::string qpath = "\"" + path + "\"";
            if (starts_with(player, "ffplay"))
            {
                return player + " -loglevel quiet -hide_banner " + qpath + " >/dev/null 2>&1";
            }
            if (player == "paplay" || player == "aplay")
            {
                return player + " " + qpath + " >/dev/null 2>&1";
            }
            if (starts_with(player, "mpg123"))
            {
                return player + " -q " + qpath + " >/dev/null 2>&1";
            }
            return player + " " + qpath + " >/dev/null 2>&1";
        }

        void log_tts_result(const char *scene, const TtsResult &ret)
        {
            const std::string tag = scene ? (std::string("[TTS] ") + scene) : "[TTS]";
            if (!ret.ok)
            {
                std::cerr << tag << " 播报失败: " << ret.error_message << std::endl;
            }
            else
            {
                std::cout << tag << " 播报成功" << std::endl;
            }
        }
    }

    TtsResult SynthesizeToFile(const std::string &base_url,
                               const std::string &text,
                               const std::string &output_path,
                               int timeout_sec)
    {
        TtsResult ret;
        if (output_path.empty())
        {
            ret.error_message = "output_path is empty";
            return ret;
        }

        ParsedUrl url;
        if (timeout_sec <= 0)
            timeout_sec = 30;

        if (!parse_http_base_url(base_url, url, ret.error_message))
            return ret;

        const std::string req_path = (url.base_path.empty() ? "" : url.base_path) + "/tts";
        const std::string body = std::string("{\"text\":\"") + json_escape(text) + "\"}";

        struct addrinfo hints;
        std::memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;

        struct addrinfo *res = nullptr;
        const std::string port_str = std::to_string(url.port);
        int gai = getaddrinfo(url.host.c_str(), port_str.c_str(), &hints, &res);
        if (gai != 0)
        {
            ret.error_message = std::string("getaddrinfo failed: ") + gai_strerror(gai);
            return ret;
        }

        int fd = -1;
        for (struct addrinfo *p = res; p != nullptr; p = p->ai_next)
        {
            fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
            if (fd < 0)
                continue;

            struct timeval tv;
            tv.tv_sec = timeout_sec;
            tv.tv_usec = 0;
            setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
            setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

            if (connect(fd, p->ai_addr, p->ai_addrlen) == 0)
                break;

            close(fd);
            fd = -1;
        }
        freeaddrinfo(res);

        if (fd < 0)
        {
            ret.error_message = std::string("connect failed: ") + std::strerror(errno);
            return ret;
        }

        std::ostringstream req;
        req << "POST " << req_path << " HTTP/1.1\r\n";
        req << "Host: " << url.host << ":" << url.port << "\r\n";
        req << "Content-Type: application/json\r\n";
        req << "Content-Length: " << body.size() << "\r\n";
        req << "Connection: close\r\n";
        req << "\r\n";
        req << body;
        const std::string req_str = req.str();

        size_t sent_total = 0;
        while (sent_total < req_str.size())
        {
            ssize_t n = send(fd, req_str.data() + sent_total, req_str.size() - sent_total, 0);
            if (n > 0)
            {
                sent_total += static_cast<size_t>(n);
                continue;
            }
            if (n < 0 && errno == EINTR)
                continue;
            ret.error_message = std::string("send failed: ") + std::strerror(errno);
            close(fd);
            return ret;
        }

        std::string raw_resp;
        if (!recv_all(fd, raw_resp, ret.error_message))
        {
            close(fd);
            return ret;
        }
        close(fd);

        std::unordered_map<std::string, std::string> headers;
        std::string resp_body;
        int status = -1;
        if (!parse_http_response(raw_resp, status, headers, resp_body, ret.error_message))
            return ret;

        ret.http_status = status;
        auto it_ct = headers.find("content-type");
        if (it_ct != headers.end())
            ret.content_type = it_ct->second;

        if (ret.http_status != 200)
        {
            ret.error_message = "http status is not 200";
            return ret;
        }

        if (ret.content_type.rfind("audio/", 0) != 0)
        {
            ret.error_message = std::string("unexpected content-type: ") + ret.content_type;
            return ret;
        }

        std::ofstream ofs(output_path, std::ios::binary);
        if (!ofs.is_open())
        {
            ret.error_message = std::string("open output failed: ") + output_path;
            return ret;
        }
        ofs.write(resp_body.data(), static_cast<std::streamsize>(resp_body.size()));
        if (!ofs.good())
        {
            ret.error_message = "write output failed";
            return ret;
        }

        ret.ok = true;
        ret.output_path = output_path;
        ret.audio_bytes = resp_body.size();
        return ret;
    }

    TtsResult SpeakText(const std::string &text,
                        const std::string &base_url,
                        int timeout_sec)
    {
        const auto now = std::chrono::system_clock::now();
        const auto ts = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
        const std::string tmp_base = "/tmp/ti5_tts_" + std::to_string(ts);

        // 先写成 .bin，拿到 content-type 后再按扩展名重命名
        TtsResult ret = SynthesizeToFile(base_url, text, tmp_base + ".bin", timeout_sec);
        if (!ret.ok)
            return ret;

        const std::string ext = guess_ext(ret.content_type);
        const std::string final_path = tmp_base + ext;
        if (::rename(ret.output_path.c_str(), final_path.c_str()) == 0)
            ret.output_path = final_path;

        const std::string player = find_player_cmd();
        if (player.empty())
        {
            ret.ok = false;
            ret.error_message = "未找到可用播放器(ffplay/paplay/aplay/mpg123)";
            return ret;
        }

        const std::string cmd = make_quiet_play_command(player, ret.output_path);
        const int play_rc = std::system(cmd.c_str());
        if (play_rc != 0)
        {
            ret.ok = false;
            ret.error_message = "音频播放失败";
            return ret;
        }
        return ret;
    }

    void SpeakTextWithLog(const std::string &text,
                          const std::string &base_url,
                          int timeout_sec,
                          const char *scene)
    {
        log_tts_result(scene, SpeakText(text, base_url, timeout_sec));
    }

    void SpeakTextAsync(std::string text,
                        const std::string &base_url,
                        int timeout_sec,
                        const char *scene)
    {
        std::thread(
            [t = std::move(text), base_url = std::string(base_url), timeout_sec, scene]()
            {
                SpeakTextWithLog(t, base_url, timeout_sec, scene);
            })
            .detach();
    }

    namespace
    {
        const char kWelcomeScene[] =
            "欢迎体验爱仕达无人售卖区，多款精美钛杯随心选，快来挑选 你的心仪款吧～";
        const char kAfterOrder[] =
            "哇，眼光超棒！这款杯子超受欢迎的～ 请稍作等候，马上为你取杯";
        const char kCupTaken[] = "杯子已成功取出，颜值质感双在线，超赞的";
        const char kDeliveryDone[] =
            "你的专属钛杯已送达～ 愿这杯美好伴你日日舒心，下次再来呀！";
    }

    void SpeakWelcomeSceneAsync()
    {
        SpeakTextAsync(kWelcomeScene, "http://127.0.0.1:4990", 30, "欢迎语");
    }

    void SpeakAfterOrderAsync()
    {
        SpeakTextAsync(kAfterOrder, "http://127.0.0.1:4990", 30, "下单确认");
    }

    void SpeakCupTakenAsync()
    {
        SpeakTextAsync(kCupTaken, "http://127.0.0.1:4990", 30, "取杯");
    }

    void SpeakDeliveryDoneAsync()
    {
        SpeakTextAsync(kDeliveryDone, "http://127.0.0.1:4990", 30, "送达完成");
    }
}
