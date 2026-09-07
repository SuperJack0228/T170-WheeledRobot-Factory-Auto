#include "Ti5_socketcan.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace
{
thread_local bool t_can_io_watch = false;
std::atomic<bool> g_can_io_fault{false};
std::atomic<bool> g_hw_abort{false};
std::atomic<int> g_can_send_fail_streak{0};
constexpr int kCanSendFailStreakAbort = 3;
std::mutex g_can_io_msg_mu;
std::string g_can_io_msg;

void can_io_set_msg(const std::string &msg)
{
    std::lock_guard<std::mutex> lock(g_can_io_msg_mu);
    g_can_io_msg = msg;
}

void note_can_send_ok()
{
    if (!t_can_io_watch)
        return;
    g_can_send_fail_streak.store(0);
}

void note_can_send_fail(int channel, uint32_t can_id)
{
    if (!t_can_io_watch)
        return;
    const int n = g_can_send_fail_streak.fetch_add(1) + 1;
    if (n < kCanSendFailStreakAbort)
        return;
    std::ostringstream oss;
    oss << "CAN 发送失败 can" << channel << " id=" << can_id << " 连续" << n << "次";
    can_io_set_msg(oss.str());
    g_can_io_fault.store(true);
}

void note_can_recv_fail(int channel, uint32_t can_id)
{
    if (!t_can_io_watch)
        return;
    std::ostringstream oss;
    oss << "CAN 读取超时 can" << channel << " id=" << can_id;
    can_io_set_msg(oss.str());
    g_can_io_fault.store(true);
}
} // namespace

bool can_io_faulted()
{
    return g_can_io_fault.load();
}

std::string can_io_fault_message()
{
    std::lock_guard<std::mutex> lock(g_can_io_msg_mu);
    return g_can_io_msg.empty() ? "CAN 收发失败" : g_can_io_msg;
}

void can_io_fault_clear()
{
    g_can_io_fault.store(false);
    g_can_send_fail_streak.store(0);
    std::lock_guard<std::mutex> lock(g_can_io_msg_mu);
    g_can_io_msg.clear();
}

void can_io_watch_begin()
{
    t_can_io_watch = true;
}

void can_io_watch_end()
{
    t_can_io_watch = false;
}

bool hardware_abort_requested()
{
    return g_hw_abort.load();
}

void hardware_abort_request()
{
    g_hw_abort.store(true);
}

void hardware_abort_clear()
{
    g_hw_abort.store(false);
}

void hardware_abort_sleep_ms(int ms)
{
    if (ms <= 0)
        return;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (g_hw_abort.load())
            return;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

std::array<int, CAN_CHANNEL_COUNT> can_sockets = {-1, -1, -1, -1, -1, -1};

 int left_h_id = -1, left_m_id = -1, left_l_id = -1, right_h_id = -1, right_m_id = -1, right_l_id = -1;

// int left_h_id = 3,
//     left_m_id = 3,
//     left_l_id = 3,
//     right_h_id = 2,
//     right_m_id = 2,
//     right_l_id = 2;


    // int head_id=1;
    // int waist_id=0;

      int head_id=-1;
    int waist_id=-1;



int convertHexArrayToDecimal2(const uint8_t hexArray[4])
{

    int result = 0;
    for (int i = 0; i < 4; i++)
        result = (result << 8) | hexArray[i];

    // 处理32位有符号数（补码转换）
    if (result > 0x7FFFFFFF)
        result -= 0x100000000;

    return result;
}

bool set_can_bitrate(const std::string &ifname, uint32_t bitrate)
{
    std::string cmd = "sudo ip link set " + ifname + " down && " +
                      "sudo ip link set " + ifname +
                      " up type can bitrate " + std::to_string(bitrate);

    std::cout << "Executing: " << cmd << std::endl;
    int ret = system(cmd.c_str());
    if (ret != 0)
    {
        std::cerr << "Command failed with code: " << ret << std::endl;
        return false;
    }
    return true;
}

bool init_can_channel(int channel, const CAN_CONFIG &config)
{
    if (channel < 0 || channel >= CAN_CHANNEL_COUNT)
    {
        std::cerr << "Invalid channel number: " << channel << std::endl;
        return false;
    }

    std::string ifname = "can" + std::to_string(channel);

    if (!set_can_bitrate(ifname, config.Bitrate))
    {
        std::cerr << "Failed to set bitrate for " << ifname << std::endl;
        return false;
    }

    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0)
    {
        perror(("Socket creation failed for " + ifname).c_str());
        return false;
    }

    if (config.Filter)
    {
        struct can_filter rfilter;
        rfilter.can_id = config.AccCode;
        rfilter.can_mask = config.AccMask;

        if (setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0)
        {
            perror("Failed to set CAN filter");
            close(sock);
            return false;
        }
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, ifname.c_str());
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0)
    {
        perror(("Interface index get failed for " + ifname).c_str());
        close(sock);
        return false;
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror(("Socket bind failed for " + ifname).c_str());
        close(sock);
        return false;
    }

    int loopback = config.Mode ? 1 : 0;
    if (setsockopt(sock, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) < 0)
    {
        perror("Failed to set loopback mode");
        close(sock);
        return false;
    }

    int flags = fcntl(sock, F_GETFL, 0);
    if (flags == -1)
    {
        perror("Failed to get socket flags");
        close(sock);
        return false;
    }
    if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) == -1)
    {
        perror("Failed to set non-blocking mode");
        close(sock);
        return false;
    }

    // if (fcntl(sock, F_SETFL, 0) == -1)
    // {
    //     perror("Failed to set blocking mode");
    //     close(sock);
    //     return false;
    // }

    can_sockets[channel] = sock;
    std::cout << "Successfully initialized " << ifname
              << " with bitrate=" << config.Bitrate / 1000 << "kbps"
              << ", filter=0x" << std::hex << config.AccCode
              << ", mask=0x" << config.AccMask << std::dec << std::endl;

    return true;
}

bool init_socketcan()
{
    CAN_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0;
    config.Filter = 1;
    config.Bitrate = 1000000;
    config.Mode = 0;

    for (int i = 0; i < CAN_CHANNEL_COUNT; i++)
    {
        if (!init_can_channel(i, config))
        {
            std::cerr << "Failed to initialize channel " << i << std::endl;
            close_can_channels();
            return false;
        }
    }
    cout << endl;
    cout << endl;
    cout << endl;
    cout << endl;
    cout << endl;

    return true;
}

bool init_socketcan_can2()
{
    CAN_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0;
    config.Filter = 1;
    config.Bitrate = 1000000;
    config.Mode = 0;

  
        if (!init_can_channel(2, config))
        {
            std::cerr << "Failed to initialize channel " << 2 << std::endl;
            close_can_channels2();
            return false;
        }
    
    cout << endl;
    cout << endl;
    cout << endl;
    cout << endl;
    cout << endl;

    return true;
}

void close_can_channels()
{
    for (int i = 0; i < CAN_CHANNEL_COUNT; i++)
    {
        if (can_sockets[i] >= 0)
        {
            close(can_sockets[i]);
            can_sockets[i] = -1;
            std::cout << "Closed channel can" << i << std::endl;
        }
    }
}

void close_can_channels2()
{
    
        if (can_sockets[2] >= 0)
        {
            close(can_sockets[2]);
            can_sockets[2] = -1;
            std::cout << "Closed channel can" << 2 << std::endl;
        }
    
}

bool send_can_frame(int channel, uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    // if (channel < 0 || channel >= CAN_CHANNEL_COUNT)
    // {
    //     std::cerr << "Invalid channel number: " << channel << std::endl;
    //     return false;
    // }

    // if (can_sockets[channel] < 0)
    // {
    //     std::cerr << "Channel " << channel << " not initialized" << std::endl;
    //     return false;
    // }

    // if (dlc > 8)
    // {
    //     std::cerr << "DLC cannot be greater than 8" << std::endl;
    //     return false;
    // }

    struct can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = dlc;
    memcpy(frame.data, data, dlc);

    int bytes_sent = write(can_sockets[channel], &frame, sizeof(frame));

    if (bytes_sent != sizeof(frame))
    {
        // perror("CAN frame send failed");
        return false;
    }

    return true;
}

bool receive_can_frame_timeout(int channel, uint8_t *data)
{

    // if (can_sockets[channel] < 0)
    // {
    //     std::cerr << "Channel " << channel << " not initialized" << std::endl;
    //     return false;
    // }

    usleep(200);
    //  3. 直接尝试读取数据
    can_frame frame;

    int bytes_read = read(can_sockets[channel], &frame, sizeof(frame));

    // if(channel!=5)
    // {
    //     cout <<"bytes_read" <<bytes_read <<endl;
    // }

    // 5. 处理结果
    if (bytes_read == sizeof(frame))
    {

        memcpy(data, frame.data, frame.can_dlc);
        return true;
    }
    else if (bytes_read == -1 && errno == EAGAIN)
    {
        // cout <<"bytes_read" <<bytes_read <<endl;

        return false; // 无数据可读
    }

    else
    {
        perror("CAN frame receive failed");
        return false;
    }

    return false;
}

void toIntArray(int number, uint8_t *res, int size)
{
    unsigned int unsignedNumber = static_cast<unsigned int>(number);
    for (int i = 0; i < size; ++i)
    {
        res[i] = unsignedNumber & 0xFF;
        unsignedNumber >>= 8;
    }
}

int socketcan_sendcommand(int channel, int cannum, uint32_t *can_idlist, uint8_t command, int *data)
{
    int flag = 0;

    for (int i = 0; i < cannum; i++)
    {
        uint8_t combinedData[5] = {0};

        // int intValue = static_cast<int>(0 / 360.0 * 4 * 65536);

        combinedData[0] = command;

        uint8_t intBytes[4];
        toIntArray(data[i], intBytes, 4);

        for (int j = 1; j < 5; j++)
        {
            combinedData[j] = intBytes[j - 1];
        }

        if (send_can_frame(channel, can_idlist[i], combinedData, 5))
        {
            note_can_send_ok();
        }
        else
        {
             std::cout << "\033[31msend_mode sent failed on can" << channel << "___Frame sent failed on canid" << can_idlist[i] << "\033[0m" << std::endl;
            note_can_send_fail(channel, can_idlist[i]);
            flag = -1;
        }
        usleep(14);
    }
    if (flag == -1)
    {
        return -1;
    }
    return 1;
    
}

int socketcan_sendsimplecommand(int channel, int cannum, uint32_t *can_idlist, uint8_t command, int *data)
{
    int flag = 0;

    for (int i = 0; i < cannum; i++)
    {
        uint8_t combinedData[1] = {0};

        combinedData[0] = command;

        // usleep(30);

        if (send_can_frame(channel, can_idlist[i], combinedData, 1))
        {
            note_can_send_ok();
        }
        else
        {
            std::cout << "Frame sent faild on can" << channel << "___Frame sent faild on canid" << can_idlist[i] << std::endl;
            note_can_send_fail(channel, can_idlist[i]);
            flag = -1;
        }

        if(command == 11)
        {
            return 0;
        }

        uint8_t recv_data[4];
        int cnt = 11;

        while ((!(receive_can_frame_timeout(channel, recv_data))) && cnt)
        {
            if (hardware_abort_requested())
                return -1;
            cnt--;
        }
        if (cnt != 0)
        {
            // 1. 将 recv_data 按小端序排列（recv_data[0]是接收到的第一个字节，作为最低字节）
            uint8_t hexArray[4] = {recv_data[4], recv_data[3], recv_data[2], recv_data[1]};

            // 2. 调用转换函数
            data[i] = convertHexArrayToDecimal2(hexArray);
        }
        else
        {
            std::cout << "\033[1;31mcan can can can  recvie  recvie recvie failed on can" << channel 
          << "can can can recvie recvie recvie recvie recvie failed on canid" 
          << can_idlist[i] << "\033[0m" << std::endl;
            note_can_recv_fail(channel, can_idlist[i]);
            flag = -1;
        }

        // std::cout << "cnn: " << cnt << "次" << std::endl;

        // if ((receive_can_frame_timeout(channel, recv_data)))
        // {
        //     // 1. 将 recv_data 按小端序排列（recv_data[0]是接收到的第一个字节，作为最低字节）
        //     uint8_t hexArray[4] = {recv_data[4], recv_data[3], recv_data[2], recv_data[1]};

        //     // 2. 调用转换函数
        //     data[i] = convertHexArrayToDecimal2(hexArray);
        // }
        // else
        // {
        //     std::cout << "\033[31mFrame sent failed on can" << channel << "___Frame sent failed on canid" << can_idlist[i] << "\033[0m" << std::endl;
        // }
    }
    if (flag == -1)
    {
        return -1;
    }
    return 1;
}

bool select_bind(int &hand_id, int id, string name)
{
    uint8_t data[4];

    int flag_id = 0;

    uint8_t combinedData[1] = {3};

    while (1)
    {
        usleep(3000);
        if (hand_id == -1 && send_can_frame(flag_id, id, combinedData, 1))
        {
            // cout << "绑定中" << endl;
            usleep(3900);
            int cnt = 10;
            while ((!(receive_can_frame_timeout(flag_id, data))) && cnt)
            {
                cnt--;
            }
            if (cnt != 0)
            {
                hand_id = flag_id;
                cout << "successfull bind->" << name << " = " << flag_id << endl;
                break;
            }
        }
        else
        {
            break;
        }
        if (flag_id >= 5)
        {
            flag_id = 0;
        }
        flag_id++;
    }
    return true;
}

void hand_socketid_bind()
{

    // int id1 = 16;
    // int id2 = 19;
    // int id3 = 21;
    // int id4 = 23;
    // int id5 = 26;
    // int id6 = 28;

    int id1 = 23;
    int id2 = 26;
    int id3 = 28;
    int id4 = 16;
    int id5 = 19;
    int id6 = 21;

    select_bind(left_h_id, id1, "left_h_id");
    select_bind(left_m_id, id2, "left_m_id");
    select_bind(left_l_id, id3, "left_l_id");
    select_bind(right_h_id, id4, "right_h_id");
    select_bind(right_m_id, id5, "right_m_id");
    select_bind(right_l_id, id6, "right_l_id");

     select_bind(waist_id, 2, "waist_id");
    select_bind(head_id, 31, "head_id");
   
}

void get_motor_position(int *data, int a)
{
    if (a == 1)
    {
        // 定义CAN ID列表
        uint32_t can_idlist_h[2] = {16, 17};
        uint32_t can_idlist_m[2] = {18, 19};
        uint32_t can_idlist_l[3] = {20, 21, 22};

        // // 创建线程容器
        // std::vector<std::thread> threads;

        // // 线程1：高位指令
        // threads.emplace_back([&]()
        //                      { socketcan_sendsimplecommand(right_h_id, 2, can_idlist_h, 8, data); });

        // // 线程2：中位指令
        // threads.emplace_back([&]()
        //                      { socketcan_sendsimplecommand(right_m_id, 2, can_idlist_m, 8, data + 2); });

        // // 线程3：低位指令
        // threads.emplace_back([&]()

        //                      { socketcan_sendsimplecommand(right_l_id, 3, can_idlist_l, 8, data + 4); });

        // // 等待所有线程完成
        // for (auto &t : threads)
        // {
        //     if (t.joinable())
        //     {
        //         t.join();
        //     }
        // }

        socketcan_sendsimplecommand(right_h_id, 2, can_idlist_h, 8, data);

        socketcan_sendsimplecommand(right_m_id, 2, can_idlist_m, 8, data + 2);

        // 线程3：低位指令
        socketcan_sendsimplecommand(right_l_id, 3, can_idlist_l, 8, data + 4);
    }
    else
    {
        uint32_t can_idlist_h[2] = {23, 24};
        uint32_t can_idlist_m[2] = {25, 26};
        uint32_t can_idlist_l[3] = {27, 28, 29};

        // std::vector<std::thread> threads;

        // threads.emplace_back([&]()
        //                      { socketcan_sendsimplecommand(left_h_id, 2, can_idlist_h, 8, data); });

        // threads.emplace_back([&]()
        //                      { socketcan_sendsimplecommand(left_m_id, 2, can_idlist_m, 8, data + 2); });

        // threads.emplace_back([&]()
        //                      { socketcan_sendsimplecommand(left_l_id, 3, can_idlist_l, 8, data + 4); });

        // for (auto &t : threads)
        // {
        //     if (t.joinable())
        //     {
        //         t.join();
        //     }
        // }

        socketcan_sendsimplecommand(left_h_id, 2, can_idlist_h, 8, data);

        socketcan_sendsimplecommand(left_m_id, 2, can_idlist_m, 8, data + 2);

        // 线程3：低位指令
        socketcan_sendsimplecommand(left_l_id, 3, can_idlist_l, 8, data + 4);
    }
}

void get_motor_running_speed(int *data, int a)
{
    constexpr uint8_t kCmdReadRunSpeed = 6;
    if (a == 1)
    {
        uint32_t can_idlist_h[2] = {16, 17};
        uint32_t can_idlist_m[2] = {18, 19};
        uint32_t can_idlist_l[3] = {20, 21, 22};

        socketcan_sendsimplecommand(right_h_id, 2, can_idlist_h, kCmdReadRunSpeed, data);
        socketcan_sendsimplecommand(right_m_id, 2, can_idlist_m, kCmdReadRunSpeed, data + 2);
        socketcan_sendsimplecommand(right_l_id, 3, can_idlist_l, kCmdReadRunSpeed, data + 4);
    }
    else
    {
        uint32_t can_idlist_h[2] = {23, 24};
        uint32_t can_idlist_m[2] = {25, 26};
        uint32_t can_idlist_l[3] = {27, 28, 29};

        socketcan_sendsimplecommand(left_h_id, 2, can_idlist_h, kCmdReadRunSpeed, data);
        socketcan_sendsimplecommand(left_m_id, 2, can_idlist_m, kCmdReadRunSpeed, data + 2);
        socketcan_sendsimplecommand(left_l_id, 3, can_idlist_l, kCmdReadRunSpeed, data + 4);
    }
}

void set_motor_position(int *data, int a)
{
    if (a == 1)
    {
        uint32_t can_idlist_h[2] = {16, 17};
        uint32_t can_idlist_m[2] = {18, 19};
        uint32_t can_idlist_l[3] = {20, 21, 22};

        // // 创建线程容器
        // std::vector<std::thread> threads;

        // // 线程1：发送高位指令
        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(right_h_id, 2, can_idlist_h, 30, data); });

        // // 线程2：发送中位指令
        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(right_m_id, 2, can_idlist_m, 30, data + 2); });

        // // 线程3：发送低位指令
        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(right_l_id, 3, can_idlist_l, 30, data + 4); });

        // // 等待所有线程完成
        // for (auto &t : threads)
        // {
        //     t.join();
        // }

        socketcan_sendcommand(right_h_id, 2, can_idlist_h, 30, data);
        ;

        socketcan_sendcommand(right_m_id, 2, can_idlist_m, 30, data + 2);
        ;

        socketcan_sendcommand(right_l_id, 3, can_idlist_l, 30, data + 4);
        ;
    }
    else
    {
        uint32_t can_idlist_h[2] = {23, 24};
        uint32_t can_idlist_m[2] = {25, 26};
        uint32_t can_idlist_l[3] = {27, 28, 29};

        // std::vector<std::thread> threads;

        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(left_h_id, 2, can_idlist_h, 30, data); });

        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(left_m_id, 2, can_idlist_m, 30, data + 2); });

        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(left_l_id, 3, can_idlist_l, 30, data + 4); });

        // for (auto &t : threads)
        // {
        //     t.join();
        // }

        socketcan_sendcommand(left_h_id, 2, can_idlist_h, 30, data);
        ;

        socketcan_sendcommand(left_m_id, 2, can_idlist_m, 30, data + 2);
        ;

        socketcan_sendcommand(left_l_id, 3, can_idlist_l, 30, data + 4);
    }
}

void get_motor_speed(int *data, int a)
{

    uint8_t speed_m = 0;
    if (data[0] > 0)
    {
        speed_m = 24;
    }
    else
    {
        speed_m = 25;
    }
    if (a == 1)
    {
        // 定义CAN ID列表
        uint32_t can_idlist_h[2] = {16, 17};
        uint32_t can_idlist_m[2] = {18, 19};
        uint32_t can_idlist_l[3] = {20, 21, 22};

        // 创建线程容器
        std::vector<std::thread> threads;

        // 线程1：高位指令
        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(right_h_id, 2, can_idlist_h, speed_m, data); });

        // 线程2：中位指令
        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(right_m_id, 2, can_idlist_m, speed_m, data + 2); });

        // 线程3：低位指令
        threads.emplace_back([&]()

                             { socketcan_sendsimplecommand(right_l_id, 3, can_idlist_l, speed_m, data + 4); });

        // socketcan_sendsimplecommand(right_h_id, 2, can_idlist_h, 8, data);

        // socketcan_sendsimplecommand(right_m_id, 2, can_idlist_m, 8, data + 2);

        // // 线程3：低位指令
        // socketcan_sendsimplecommand(right_l_id, 3, can_idlist_l, 8, data + 4);

        // 等待所有线程完成
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
    }
    else
    {
        uint32_t can_idlist_h[2] = {23, 24};
        uint32_t can_idlist_m[2] = {25, 26};
        uint32_t can_idlist_l[3] = {27, 28, 29};

        std::vector<std::thread> threads;

        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(left_h_id, 2, can_idlist_h, speed_m, data); });

        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(left_m_id, 2, can_idlist_m, speed_m, data + 2); });

        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(left_l_id, 3, can_idlist_l, speed_m, data + 4); });

        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
    }
}

void set_motor_addspeed(int *data, int a)
{
    uint8_t speed_m = 0;
    if (data[0] > 0)
    {
        speed_m = 34;
    }
    else
    {
        speed_m = 35;
    }
    if (a == 1)
    {

        uint32_t can_idlist_h[2] = {16, 17};
        uint32_t can_idlist_m[2] = {18, 19};
        uint32_t can_idlist_l[3] = {20, 21, 22};

        // 创建线程容器
        std::vector<std::thread> threads;

        // 线程1：发送高位指令
        threads.emplace_back([&]()
                             { socketcan_sendcommand(right_h_id, 2, can_idlist_h, speed_m, data); });

        // 线程2：发送中位指令
        threads.emplace_back([&]()
                             { socketcan_sendcommand(right_m_id, 2, can_idlist_m, speed_m, data + 2); });

        // 线程3：发送低位指令
        threads.emplace_back([&]()
                             { socketcan_sendcommand(right_l_id, 3, can_idlist_l, speed_m, data + 4); });

        // socketcan_sendcommand(right_h_id, 2, can_idlist_h, 30, data);
        // ;

        // socketcan_sendcommand(right_m_id, 2, can_idlist_m, 30, data + 2);
        // ;

        // socketcan_sendcommand(right_l_id, 3, can_idlist_l, 30, data + 4);
        // ;

        // 等待所有线程完成
        for (auto &t : threads)
        {
            t.join();
        }
    }
    else
    {
        uint32_t can_idlist_h[2] = {23, 24};
        uint32_t can_idlist_m[2] = {25, 26};
        uint32_t can_idlist_l[3] = {27, 28, 29};

        std::vector<std::thread> threads;

        threads.emplace_back([&]()
                             { socketcan_sendcommand(left_h_id, 2, can_idlist_h, speed_m, data); });

        threads.emplace_back([&]()
                             { socketcan_sendcommand(left_m_id, 2, can_idlist_m, speed_m, data + 2); });

        threads.emplace_back([&]()
                             { socketcan_sendcommand(left_l_id, 3, can_idlist_l, speed_m, data + 4); });

        for (auto &t : threads)
        {
            t.join();
        }
    }
}

void get_motor_addspeed(int *data, int a)
{

    uint8_t speed_m = 0;
    if (data[0] > 0)
    {
        speed_m = 22;
    }
    else
    {
        speed_m = 23;
    }
    if (a == 1)
    {
        // 定义CAN ID列表
        uint32_t can_idlist_h[2] = {16, 17};
        uint32_t can_idlist_m[2] = {18, 19};
        uint32_t can_idlist_l[3] = {20, 21, 22};

        // 创建线程容器
        std::vector<std::thread> threads;

        // 线程1：高位指令
        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(right_h_id, 2, can_idlist_h, speed_m, data); });

        // 线程2：中位指令
        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(right_m_id, 2, can_idlist_m, speed_m, data + 2); });

        // 线程3：低位指令
        threads.emplace_back([&]()

                             { socketcan_sendsimplecommand(right_l_id, 3, can_idlist_l, speed_m, data + 4); });

        // socketcan_sendsimplecommand(right_h_id, 2, can_idlist_h, 8, data);

        // socketcan_sendsimplecommand(right_m_id, 2, can_idlist_m, 8, data + 2);

        // // 线程3：低位指令
        // socketcan_sendsimplecommand(right_l_id, 3, can_idlist_l, 8, data + 4);

        // 等待所有线程完成
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
    }
    else
    {
        uint32_t can_idlist_h[2] = {23, 24};
        uint32_t can_idlist_m[2] = {25, 26};
        uint32_t can_idlist_l[3] = {27, 28, 29};

        std::vector<std::thread> threads;

        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(left_h_id, 2, can_idlist_h, speed_m, data); });

        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(left_m_id, 2, can_idlist_m, speed_m, data + 2); });

        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(left_l_id, 3, can_idlist_l, speed_m, data + 4); });

        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
    }
}

void set_motor_speed(int *data, int a)
{
    uint8_t speed_m = 0;
    if (data[0] > 0)
    {
        speed_m = 36;
    }
    else
    {
        speed_m = 37;
    }
    if (a == 1)
    {

        uint32_t can_idlist_h[2] = {16, 17};
        uint32_t can_idlist_m[2] = {18, 19};
        uint32_t can_idlist_l[3] = {20, 21, 22};

        // 创建线程容器
        std::vector<std::thread> threads;

        // 线程1：发送高位指令
        threads.emplace_back([&]()
                             { socketcan_sendcommand(right_h_id, 2, can_idlist_h, speed_m, data); });

        // 线程2：发送中位指令
        threads.emplace_back([&]()
                             { socketcan_sendcommand(right_m_id, 2, can_idlist_m, speed_m, data + 2); });

        // 线程3：发送低位指令
        threads.emplace_back([&]()
                             { socketcan_sendcommand(right_l_id, 3, can_idlist_l, speed_m, data + 4); });

        // socketcan_sendcommand(right_h_id, 2, can_idlist_h, 30, data);
        // ;

        // socketcan_sendcommand(right_m_id, 2, can_idlist_m, 30, data + 2);
        // ;

        // socketcan_sendcommand(right_l_id, 3, can_idlist_l, 30, data + 4);
        // ;

        // 等待所有线程完成
        for (auto &t : threads)
        {
            t.join();
        }
    }
    else
    {
        uint32_t can_idlist_h[2] = {23, 24};
        uint32_t can_idlist_m[2] = {25, 26};
        uint32_t can_idlist_l[3] = {27, 28, 29};

        std::vector<std::thread> threads;

        threads.emplace_back([&]()
                             { socketcan_sendcommand(left_h_id, 2, can_idlist_h, speed_m, data); });

        threads.emplace_back([&]()
                             { socketcan_sendcommand(left_m_id, 2, can_idlist_m, speed_m, data + 2); });

        threads.emplace_back([&]()
                             { socketcan_sendcommand(left_l_id, 3, can_idlist_l, speed_m, data + 4); });

        for (auto &t : threads)
        {
            t.join();
        }
    }
}

void set_motor(int *data, uint8_t command, int a)
{
  
    if (a == 1)
    {

        uint32_t can_idlist_h[2] = {16, 17};
        uint32_t can_idlist_m[2] = {18, 19};
        uint32_t can_idlist_l[3] = {20, 21, 22};

        // // 创建线程容器
        // std::vector<std::thread> threads;

        // // 线程1：发送高位指令
        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(right_h_id, 2, can_idlist_h, command, data); });

        // // 线程2：发送中位指令
        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(right_m_id, 2, can_idlist_m, command, data + 2); });

        // // 线程3：发送低位指令
        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(right_l_id, 3, can_idlist_l, command, data + 4); });

        socketcan_sendcommand(right_h_id, 2, can_idlist_h, command, data);
        

        socketcan_sendcommand(right_m_id, 2, can_idlist_m, command, data + 2);
        

        socketcan_sendcommand(right_l_id, 3, can_idlist_l, command, data + 4);
        

        // // 等待所有线程完成
        // for (auto &t : threads)
        // {
        //     t.join();
        // }
    }
    else
    {
        uint32_t can_idlist_h[2] = {23, 24};
        uint32_t can_idlist_m[2] = {25, 26};
        uint32_t can_idlist_l[3] = {27, 28, 29};


        socketcan_sendcommand(left_h_id, 2, can_idlist_h, command, data);
        

        socketcan_sendcommand(left_m_id, 2, can_idlist_m, command, data + 2);
        

        socketcan_sendcommand(left_l_id, 3, can_idlist_l, command, data + 4);
        

        // std::vector<std::thread> threads;

        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(left_h_id, 2, can_idlist_h, command, data); });

        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(left_m_id, 2, can_idlist_m, command, data + 2); });

        // threads.emplace_back([&]()
        //                      { socketcan_sendcommand(left_l_id, 3, can_idlist_l, command, data + 4); });

        // for (auto &t : threads)
        // {
        //     t.join();
        // }
    }
}


void set_motor_singn_mode(int *data,uint8_t command, int a)
{
    if (a == 1)
    {
        // 定义CAN ID列表
        uint32_t can_idlist_h[2] = {16, 17};
        uint32_t can_idlist_m[2] = {18, 19};
        uint32_t can_idlist_l[3] = {20, 21, 22};

        // // 创建线程容器
        // std::vector<std::thread> threads;

        // // 线程1：高位指令
        // threads.emplace_back([&]()
        //                      { socketcan_sendsimplecommand(right_h_id, 2, can_idlist_h, 8, data); });

        // // 线程2：中位指令
        // threads.emplace_back([&]()
        //                      { socketcan_sendsimplecommand(right_m_id, 2, can_idlist_m, 8, data + 2); });

        // // 线程3：低位指令
        // threads.emplace_back([&]()

        //                      { socketcan_sendsimplecommand(right_l_id, 3, can_idlist_l, 8, data + 4); });

        // // 等待所有线程完成
        // for (auto &t : threads)
        // {
        //     if (t.joinable())
        //     {
        //         t.join();
        //     }
        // }

        socketcan_sendsimplecommand(right_h_id, 2, can_idlist_h, command, data);

        socketcan_sendsimplecommand(right_m_id, 2, can_idlist_m, command, data + 2);

        // 线程3：低位指令
        socketcan_sendsimplecommand(right_l_id, 3, can_idlist_l, command, data + 4);
    }
    else
    {
        uint32_t can_idlist_h[2] = {23, 24};
        uint32_t can_idlist_m[2] = {25, 26};
        uint32_t can_idlist_l[3] = {27, 28, 29};

        // std::vector<std::thread> threads;

        // threads.emplace_back([&]()
        //                      { socketcan_sendsimplecommand(left_h_id, 2, can_idlist_h, 8, data); });

        // threads.emplace_back([&]()
        //                      { socketcan_sendsimplecommand(left_m_id, 2, can_idlist_m, 8, data + 2); });

        // threads.emplace_back([&]()
        //                      { socketcan_sendsimplecommand(left_l_id, 3, can_idlist_l, 8, data + 4); });

        // for (auto &t : threads)
        // {
        //     if (t.joinable())
        //     {
        //         t.join();
        //     }
        // }

        socketcan_sendsimplecommand(left_h_id, 2, can_idlist_h, command, data);

        socketcan_sendsimplecommand(left_m_id, 2, can_idlist_m, command, data + 2);

        // 线程3：低位指令
        socketcan_sendsimplecommand(left_l_id, 3, can_idlist_l, command, data + 4);
    }
}



void set_motor_current_zero()
{
    int data[7]={0,0,0,0,0,0,0};
    set_motor(data, 28, 0);
    set_motor(data, 28, 1);
}
// {

//     int data[7]={9000,9000,9000,9000,9000,9000,9000};
//     int data2[7]={-9000,-9000,-9000,-9000,-9000,-9000,-9000};
//     int data3[7]={9000,9000,9000,9000,9000,9000,9000};
//     int data4[7]={-9000,-9000,-9000,-9000,-9000,-9000,-9000};

//      set_motor_speed(data, 1);
//       set_motor_speed(data2, 1);
//        set_motor_speed(data3, 0);
//         set_motor_speed(data4, 0);

// }

// sleep(5);

// {
//     int data[7]={1,0,0,0,0,0,0};
//     int data2[7]={-1,0,0,0,0,0,0};
//     int data3[7]={1,0,0,0,0,0,0};
//     int data4[7]={-1,0,0,0,0,0,0};

//      get_motor_speed(data, 1);
//       get_motor_speed(data2, 1);
//        get_motor_speed(data3, 0);
//         get_motor_speed(data4, 0);

//         cout << "r speed:" << data[0] << " " << data[1] << " " << data[2] << " " << data[3] << " " << data[4] << " " << data[5] << " " << data[6] << " " << endl;
//         cout << "r speed:" << data2[0] << " " << data2[1] << " " << data2[2] << " " << data2[3] << " " << data2[4] << " " << data2[5] << " " << data2[6] << " " << endl;
//         cout << "r speed:" << data3[0] << " " << data3[1] << " " << data3[2] << " " << data3[3] << " " << data3[4] << " " << data3[5] << " " << data3[6] << " " << endl;
//         cout << "r speed:" << data4[0] << " " << data4[1] << " " << data4[2] << " " << data4[3] << " " << data4[4] << " " << data4[5] << " " << data4[6] << " " << endl;

// }

// {

//     int data[7]={30,30,30,30,30,30,30};
//     int data2[7]={-30,-30,-30,-30,-30,-30,-30};
//     int data3[7]={30,30,30,30,30,30,30};
//     int data4[7]={-30,-30,-30,-30,-30,-30,-30};

//      set_motor_addspeed(data, 1);
//       set_motor_addspeed(data2, 1);
//        set_motor_addspeed(data3, 0);
//         set_motor_addspeed(data4, 0);

// }

// sleep(2);

// {
//     int data[7] = {1, 0, 0, 0, 0, 0, 0};
//     int data2[7] = {-1, 0, 0, 0, 0, 0, 0};
//     int data3[7] = {1, 0, 0, 0, 0, 0, 0};
//     int data4[7] = {-1, 0, 0, 0, 0, 0, 0};

//     get_motor_addspeed(data, 1);
//     get_motor_addspeed(data2, 1);
//     get_motor_addspeed(data3, 0);
//     get_motor_addspeed(data4, 0);

//     cout << "r speed:" << data[0] << " " << data[1] << " " << data[2] << " " << data[3] << " " << data[4] << " " << data[5] << " " << data[6] << " " << endl;
//     cout << "r speed:" << data2[0] << " " << data2[1] << " " << data2[2] << " " << data2[3] << " " << data2[4] << " " << data2[5] << " " << data2[6] << " " << endl;
//     cout << "r speed:" << data3[0] << " " << data3[1] << " " << data3[2] << " " << data3[3] << " " << data3[4] << " " << data3[5] << " " << data3[6] << " " << endl;
//     cout << "r speed:" << data4[0] << " " << data4[1] << " " << data4[2] << " " << data4[3] << " " << data4[4] << " " << data4[5] << " " << data4[6] << " " << endl;
// }

// sleep(10000);

// while (1)
// {

// int data2[7] = {0, 0, 0, 0, 0, 0};

// set_motor_position(data2, 1);
// set_motor_position(data2, 0);

//     sleep(4);

//     // int data3[7] = {static_cast<int>(-0.561221 * 180 / 3.14 * 4 * 65536/360),static_cast<int>(-0.203564 * 180 / 3.14 * 4 * 65536/360),static_cast<int>(1.73402 * 180 / 3.14 * 4 * 65536/360),
//     //     static_cast<int>(0.870989 * 180 / 3.14 * 4 * 65536/360),static_cast<int>(-1.53887 * 180 / 3.14 * 4 * 65536/360),
//     //     static_cast<int>(-0.0890188 * 180 / 3.14 * 4 * 65536/360),static_cast<int>(-0.221013* 180 / 3.14 * 4 * 65536/360)};
//     // int data4[7] = {static_cast<int>(-0.544108 * 180 / 3.14 * 4 * 65536/360),static_cast<int>(0.203472 * 180 / 3.14 * 4 * 65536/360),static_cast<int>(-1.77717 * 180 / 3.14 * 4 * 65536/360),
//     //     static_cast<int>(-0.87906 * 180 / 3.14 * 4 * 65536/360),static_cast<int>(1.5303 * 180 / 3.14 * 4 * 65536/360),
//     //     static_cast<int>(-0.0800856 * 180 / 3.14 * 4 * 65536/360),static_cast<int>(-0.2204776 * 180 / 3.14 * 4 * 65536/360)};

//     //     for(int i=0;i<7;i++)
//     //     {
//     //         cout << data3[i] <<endl;
//     //          cout << data4[i] <<endl;
//     //     }
//     //        // cin >> ahhh;
//     // set_motor_position(data3,1);
//     // set_motor_position(data4,0);

//  //     sleep(4);

//         //     return 0;
//         // }

//         Matrix<double, 1, 6> pos1, pos2, goal_last, pos3;

//         pos1 << 0.25, -0.3, -0.15, 0, 0, 0;
//         pos2 << 0.25, 0.4, -0.15, 0, 0, 0;
//         pos3 << 0.45, -0.15, -0.2, 0, 0, 0;

//         while (1)
//         {
//             int a;
//             cin >> a;
//             Taihu_r.moveLToPos(pos1, 1);
//             pos1[0] += 0.2;

//             cin >> a;

//             Taihu_r.moveLToPos(pos1, 1);

//             pos1[0] -= 0.2;

//             // Taihu_l.moveLToPos(pos2, 1);
//         }

//         // sleep(2);




void set_waist_motor_position(int *data)
{
   
        uint32_t can_idlist[3] = {4, 3,2};
      

       

       

        socketcan_sendcommand(waist_id, 3, can_idlist, 30, data ); 
  
   
}


void get_motor_waist_position(int *data)
{
  
      
        uint32_t can_idlist[3] = {4, 3, 2};


      

       
        socketcan_sendsimplecommand(waist_id, 3, can_idlist, 8, data );

    //    cout <<
    
  
}

namespace
{

constexpr double kMotorCntPerDeg = 65536.0 * 4.0 / 360.0;

double quintic_scalar(double t, double total_time, double start, double target)
{
    if (total_time <= 1e-6)
        return target;

    const double ratio = std::clamp(t / total_time, 0.0, 1.0);
    const double r2 = ratio * ratio;
    const double r3 = r2 * ratio;
    const double r4 = r3 * ratio;
    const double r5 = r4 * ratio;
    const double pos_term = 10.0 * r3 - 15.0 * r4 + 6.0 * r5;
    return start + (target - start) * pos_term;
}

} // namespace

int smooth_motor_move_deg(
    int channel,
    int motor_count,
    uint32_t *can_idlist,
    const double *target_angles_deg,
    double speed_deg_per_s,
    int dt_ms)
{
    if (motor_count <= 0 || can_idlist == nullptr || target_angles_deg == nullptr)
        return -1;
    if (speed_deg_per_s <= 1e-6)
        speed_deg_per_s = 10.0;
    if (dt_ms <= 0)
        dt_ms = 20;

    std::vector<int> current_cnt(static_cast<size_t>(motor_count), 0);
    if (socketcan_sendsimplecommand(channel, motor_count, can_idlist, 8, current_cnt.data()) != 1)
        return -1;

    std::vector<double> start_deg(static_cast<size_t>(motor_count));
    std::vector<double> target_deg(static_cast<size_t>(motor_count));
    std::vector<int> pos_data(static_cast<size_t>(motor_count));

    double max_delta_deg = 0.0;
    for (int i = 0; i < motor_count; ++i)
    {
        start_deg[static_cast<size_t>(i)] = motor_cnt_to_angle_deg(current_cnt[static_cast<size_t>(i)]);
        target_deg[static_cast<size_t>(i)] = target_angles_deg[i];
        max_delta_deg = std::max(
            max_delta_deg,
            std::abs(target_deg[static_cast<size_t>(i)] - start_deg[static_cast<size_t>(i)]));
    }

    if (max_delta_deg < 0.01)
    {
        for (int i = 0; i < motor_count; ++i)
            pos_data[static_cast<size_t>(i)] = motor_angle_deg_to_cnt(target_deg[static_cast<size_t>(i)]);
        socketcan_sendcommand(channel, motor_count, can_idlist, 30, pos_data.data());
        return 0;
    }

    const double total_time_s = max_delta_deg / speed_deg_per_s;
    const double dt_s = static_cast<double>(dt_ms) / 1000.0;

    double t = 0.0;
    while (t < total_time_s)
    {
        for (int i = 0; i < motor_count; ++i)
        {
            const double angle_deg = quintic_scalar(
                t,
                total_time_s,
                start_deg[static_cast<size_t>(i)],
                target_deg[static_cast<size_t>(i)]);
            pos_data[static_cast<size_t>(i)] = motor_angle_deg_to_cnt(angle_deg);
        }
        socketcan_sendcommand(channel, motor_count, can_idlist, 30, pos_data.data());
        if (hardware_abort_requested() || can_io_faulted())
            return -1;
        usleep(static_cast<useconds_t>(dt_ms * 1000));
        t += dt_s;
    }

    for (int i = 0; i < motor_count; ++i)
        pos_data[static_cast<size_t>(i)] = motor_angle_deg_to_cnt(target_deg[static_cast<size_t>(i)]);
    socketcan_sendcommand(channel, motor_count, can_idlist, 30, pos_data.data());
    return 0;
}


void motor_speed_change()
{
    int data[7] = {};

    // set_motor_position(data, 0);
    // set_motor_position(data, 1);

    int speed_data[7] = {1, 1, 1, 1, 1, 1, 1};

    get_motor_speed(speed_data, 1);

    for (int i = 0; i < 7; i++)
    {
        cout << " " << speed_data[i];
    }
    cout << endl;

    get_motor_speed(speed_data, 0);

    for (int i = 0; i < 7; i++)
    {
        cout << " " << speed_data[i];
    }
    cout << endl;

    speed_data[0] = -1;

    get_motor_speed(speed_data, 1);

    for (int i = 0; i < 7; i++)
    {
        cout << " " << speed_data[i];
    }
    cout << endl;

    get_motor_speed(speed_data, 0);

    for (int i = 0; i < 7; i++)
    {
        cout << " " << speed_data[i];
    }
    cout << endl;

    int speed[7] = {2500, 2500, 2500, 2500, 2500, 2500, 2500};
    int speed2[7] = {-2500, -2500, -2500, -2500, -2500, -2500, -2500};

    set_motor_speed(speed, 1);
    set_motor_speed(speed2, 1);
    set_motor_speed(speed, 0);
    set_motor_speed(speed2, 0);

    speed_data[0] = 1;
    get_motor_speed(speed_data, 1);

    for (int i = 0; i < 7; i++)
    {
        cout << " " << speed_data[i];
    }
    cout << endl;

    get_motor_speed(speed_data, 0);

    for (int i = 0; i < 7; i++)
    {
        cout << " " << speed_data[i];
    }
    cout << endl;

    speed_data[0] = -1;

    get_motor_speed(speed_data, 1);

    for (int i = 0; i < 7; i++)
    {
        cout << " " << speed_data[i];
    }
    cout << endl;

    get_motor_speed(speed_data, 0);

    for (int i = 0; i < 7; i++)
    {
        cout << " " << speed_data[i];
    }
    cout << endl;

    sleep(1);

    std::cout << "------------------------------" << std::endl;
}



void rev_motor_error( int a)
{
    int data[7];

    uint8_t speed_m = 11;
   
    if (a == 1)
    {
        // 定义CAN ID列表
        uint32_t can_idlist_h[2] = {16, 17};
        uint32_t can_idlist_m[2] = {18, 19};
        uint32_t can_idlist_l[3] = {20, 21, 22};

        // 创建线程容器
        std::vector<std::thread> threads;

        // 线程1：高位指令
        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(right_h_id, 2, can_idlist_h, speed_m, data); });

        // 线程2：中位指令
        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(right_m_id, 2, can_idlist_m, speed_m, data + 2); });

        // 线程3：低位指令
        threads.emplace_back([&]()

                             { socketcan_sendsimplecommand(right_l_id, 3, can_idlist_l, speed_m, data + 4); });

        // socketcan_sendsimplecommand(right_h_id, 2, can_idlist_h, 8, data);

        // socketcan_sendsimplecommand(right_m_id, 2, can_idlist_m, 8, data + 2);

        // // 线程3：低位指令
        // socketcan_sendsimplecommand(right_l_id, 3, can_idlist_l, 8, data + 4);

        // 等待所有线程完成
        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
    }
    else
    {
        uint32_t can_idlist_h[2] = {23, 24};
        uint32_t can_idlist_m[2] = {25, 26};
        uint32_t can_idlist_l[3] = {27, 28, 29};

        std::vector<std::thread> threads;

        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(left_h_id, 2, can_idlist_h, speed_m, data); });

        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(left_m_id, 2, can_idlist_m, speed_m, data + 2); });

        threads.emplace_back([&]()
                             { socketcan_sendsimplecommand(left_l_id, 3, can_idlist_l, speed_m, data + 4); });

        for (auto &t : threads)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
    }
}


