#include "internal/state.h"
#include "t170/hardware/device_bus.h"
#include "t170/vision/debug.h"
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <thread>
#include "t170/vision/python_runtime.h"
#include "t170/common/paths.h"
#include <ifaddrs.h>
#include <sstream>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>

namespace t170::app {
using namespace detail;
namespace {
void startup_summary(const Options &options) {
    std::string ip="127.0.0.1";
    ifaddrs *interfaces=nullptr;
    if(getifaddrs(&interfaces)==0) {
        for(auto *p=interfaces;p;p=p->ifa_next) {
            if(!p->ifa_addr || p->ifa_addr->sa_family!=AF_INET)continue;
            const auto *a=reinterpret_cast<const sockaddr_in *>(p->ifa_addr);
            if((ntohl(a->sin_addr.s_addr)>>24)==127)continue;
            char address[INET_ADDRSTRLEN]{};
            if(inet_ntop(AF_INET,&a->sin_addr,address,sizeof(address))) {ip=address;break;}
        }
        freeifaddrs(interfaces);
    }
    const auto *workspace=std::getenv("T170_WORKSPACE");
    const std::string model=g_arm_l?g_arm_l->definition().model_id:
        g_arm_r?g_arm_r->definition().model_id:"unknown";
    std::ostringstream out;
    out<<"\n[启动就绪] 机器人="<<g_move_cfg.chassis.sequence_number<<" 型号="<<model
       <<" 模式="<<(options.mode==Mode::Maintenance?"维护":"调度")
       <<" 手相机修正="<<(g_move_cfg.cameras.hand_descent_correct?"开启":"关闭（使用示教XY）")
       <<" 底盘="<<g_move_cfg.chassis.host<<"\n"
       <<"[快捷信息] 参数="<<(workspace?workspace:project_root_dir())<<"/config"
       <<" 日志="<<t170::paths::log_directory().string()
       <<" 网页默认入口=http://"<<ip<<":8080/"
       <<" 本机命令=127.0.0.1:"<<options.port<<"\n";
    std::cerr<<out.str()<<std::flush;
}
}
int run(int argc, char **argv)
{
    try
    {
        Options options;
        try {options=parse_options(argc,argv);}
        catch(const std::exception &ex) {std::cerr<<ex.what()<<'\n'<<usage();return 2;}
        if(options.help){std::cout<<usage();return 0;}
        state().mode=options.mode;
        if(options.list_commands){std::cout<<capabilities_json()<<'\n';return 0;}
        const int port=options.port;
        const bool boot_dispatch=options.mode==Mode::Production;
        install_app_sigint_handler();
        if (boot_dispatch)
            orch::boot_announce("initialize");
        std::string error;
        if (!init_hardware(error))
        {
            std::cerr << "[t170_factory] 初始化失败: " << error << "\n";
            orch::boot_announce("failure");
            shutdown_process(1);
        }
        if (boot_dispatch)
            orch::boot_announce("initialize_finish");

        const int server = ::socket(AF_INET, SOCK_STREAM, 0);
        int yes = 1;
        setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(static_cast<uint16_t>(port));
        inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
        if (::bind(server, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0 || listen(server, 8) != 0)
        {
            std::cerr << "[t170_factory] 无法监听 127.0.0.1:" << port << "\n";
            if (server >= 0)
                ::close(server);
            orch::boot_announce("failure");
            shutdown_process(1);
        }

        orch::start_chassis_status_reporter();
        std::cerr << "T170_FACTORY_READY port=" << port << " chassis=" << g_move_cfg.chassis.host
                  << " belt=" << g_move_cfg.chassis.belt_station
                  << " out=" << g_move_cfg.chassis.out_station
                  << " mode=" << mode_name(options.mode)
                  << (boot_dispatch ? " dispatch=boot\n" : "\n");
        if (boot_dispatch)
            start_boot_dispatch();
        startup_summary(options);
        with_python_gil_released([&] {
            while (!app_stop_requested())
            {
                pollfd pfd{};
                pfd.fd = server;
                pfd.events = POLLIN;
                const int pr = ::poll(&pfd, 1, 200);
                if (pr < 0)
                {
                    if (errno == EINTR)
                        continue;
                    std::cerr << "[t170_factory] poll 失败\n";
                    break;
                }
                if (pr == 0 || app_stop_requested())
                    continue;
                const int client = ::accept(server, nullptr, nullptr);
                if (client >= 0)
                    std::thread(handle_client, client).detach();
            }
        });
        ::close(server);
        shutdown_process(app_stop_requested() ? 130 : 0);
    }
    catch (const std::exception &ex)
    {
        std::cerr << "[t170_factory] 未捕获异常: " << ex.what() << "\n";
        shutdown_process(1);
    }
    catch (...)
    {
        std::cerr << "[t170_factory] 未捕获异常\n";
        shutdown_process(1);
    }
}
}
