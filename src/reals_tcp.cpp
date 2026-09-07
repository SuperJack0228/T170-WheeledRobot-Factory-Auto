#include "reals_tcp.h"
#include "Ti5_socketcan.h"
#include "waist.h"

tcp::socket *g_socket = nullptr;

tcp::socket reals_tcp_init()
{
    boost::asio::io_service io;
    tcp::socket socket(io);

    // 连接 Python 服务器
    socket.connect(tcp::endpoint(boost::asio::ip::address::from_string("127.0.0.1"), 12345));

    std::cout << "✅ 已连接到 Python 服务器" << std::endl;
    return socket;
}

std::string reals_tcp_trsmit(tcp::socket &socket, std::string target)
{

    // 发送指令到 Python
    boost::asio::write(socket, boost::asio::buffer(target + "\n"));

    // 接收 Python 的响应（原始模式）
    boost::asio::streambuf buf;
    boost::system::error_code ec;

    // 读取所有可用数据（不等待换行符）
    size_t len = boost::asio::read(socket, buf, boost::asio::transfer_at_least(1), ec);

    if (ec && ec != boost::asio::error::eof)
    {
        throw boost::system::system_error(ec);
    }

    // 打印原始接收数据
    std::string raw_data(boost::asio::buffers_begin(buf.data()),
                         boost::asio::buffers_begin(buf.data()) + len);

    std::istream is(&buf);
    std::string response;
    while (std::getline(is, response))
    {
        // std::cout << "解析后的响应: " << response << std::endl;
    }

    return response;
}

int pos_move(double goal_last[3], Robot_Arm &Taihu)
{

    Matrix<double, 1, 6> posd = {goal_last[0], goal_last[1], goal_last[2], 0, 0, 0};

    sleep(1);
    return arm_line_move(Taihu, posd, 0.2);
}

int position_con(Robot_Arm &Taihu)
{
    cout << "开始移动操作" << endl;
    Matrix<double, 1, 6> current_hand_pos = arm_get_tcp_pos(Taihu);
    double curr_pos[6] = {current_hand_pos(0), current_hand_pos(1), current_hand_pos(2), 0, 0, 0};
    cout << "当前位置:" << current_hand_pos(0) << current_hand_pos(1) << current_hand_pos(2) << endl;

    while (true)
    {
        int j;
        cin >> j;
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "j:" << j << endl;
        if (j == 1)
        {
            curr_pos[0] += 0.05;
            int ret = pos_move(curr_pos, Taihu);
            if (ret != 0)
            {
                curr_pos[0] -= 0.05;
            }
        }
        else if (j == 2)
        {
            curr_pos[0] -= 0.05;
            int ret = pos_move(curr_pos, Taihu);
            if (ret != 0)
            {
                curr_pos[0] += 0.05;
            }
        }
        else if (j == 4)
        {
            curr_pos[1] += 0.05;
            int ret = pos_move(curr_pos, Taihu);
            if (ret != 0)
            {
                curr_pos[1] -= 0.05;
            }
        }
        else if (j == 5)
        {
            curr_pos[1] -= 0.05;
            int ret = pos_move(curr_pos, Taihu);
            if (ret != 0)
            {
                curr_pos[1] += 0.05;
            }
        }
        else if (j == 7)
        {
            curr_pos[2] += 0.02;
            int ret = pos_move(curr_pos, Taihu);
            if (ret != 0)
            {
                curr_pos[2] -= 0.02;
            }
        }
        else if (j == 8)
        {
            curr_pos[2] -= 0.02;
            int ret = pos_move(curr_pos, Taihu);
            if (ret != 0)
            {
                curr_pos[2] += 0.02;
            }
        }
        else
        {
            break;
        }
    }
    return 0;
}

int mech(Robot_Arm &Taihu, Robot_Arm &Taihu_l)
{

    //     {

    Matrix<double, 1, 7> q_j_r;
    q_j_r << 0.000719053, -1.56991, 1.57187, -0.000167779, -1.57156, 0.00107858, 0.000527306;

    Matrix<double, 1, 7> q_j_l;
    q_j_l << 0.000191748, 1.56984, -1.5696, -0.000383495, 1.57204, 0.00093477, -0.000479369;

    arm_joint_move(Taihu, q_j_r);
    arm_joint_move(Taihu_l, q_j_l);
    // }

    // int data[7] = {0,0,0,0,0,0,0};

    // set_motor_position(data, 0);
    // set_motor_position(data, 1);

    return 0;
}

int start_pos(Robot_Arm &Taihu_r, Robot_Arm &Taihu_l, int flag)
{

    Eigen::Matrix<double, 1, 6> posup_down_3_layer;

    posup_down_3_layer << -0.132502, 4.36259e-17, 0.622464, -1.5708, -1.56703, 3.14159;

    int waist_p[1] = {0 * 65536 * 4 / 360};
    uint32_t waist_canID[] = {1};

    socketcan_sendcommand(waist_id, 1, waist_canID, 30, waist_p);

    sleep(2);

    WaistRobot Ti5_waist;

    posup_down_3_layer(0) = 0.13;

    Ti5_waist.moveLToPos(posup_down_3_layer, 0.1);

    {
        Matrix<double, 1, 6> posr, posl, goal_last, goal_last_copy;

        posr << 0.45, -0.36, -0.15, -90 * rad, 0, 0;
        posl << 0.45, 0.36, -0.15, 90 * rad, 0, 0;
        arm_dual_line_move(Taihu_r, posr, Taihu_l, posl, 0.2);
    }
    Matrix<double, 1, 7> q_s_j_r2 = {-0.66117, -0.394449, 0.183646, 2.18791, 0.0589864, 0.123749, -0.0720971};
    Matrix<double, 1, 7> q_s_j_l2 = {-0.932517, 0.310511, -0.381362, 2.27168, -0.255743, 0.155867, 0.0662248};

    // Matrix<double, 1, 7> q_s_j_r = {-0.761221, -0.63564, 0, 1.290989, 0, -0.0, 0.0};
    // Matrix<double, 1, 7> q_s_j_l = {-0.744108, 0.63472, 0, 1.29906, 0, -0.0, 0.0};

    // posup_down_3_layer(0) = -0.13;

    // Ti5_waist.moveLToPos(posup_down_3_layer, 0.1);

    return 0;
}

int zhuaqu(Matrix<double, 1, 6> &goal_last, Robot_Arm &Taihu, int flag, std::string target, double lin)
{
    Matrix<double, 1, 6> pos1, pos2;

    int juli = 0;

    pos1 << 0.24, -0.4, -0.15, 0, 0, 0;
    pos2 << 0.24, 0.4, -0.15, 0, 0, 0;

    if (flag == 1)
    {
        tiger_hand(right_a);
    }
    else
    {
        tiger_hand(left_a);
    }

    goal_last[0] += readPiancha("1");
    juli = 1;

    int stop;

    int ret1;

    // cin >> stop;
    // cin.ignore(numeric_limits<streamsize>::max(), '\n');

    if (flag == 2) // left
    {

        ret1 = moveL(Taihu, goal_last, 0.2);

        if (ret1 == -1)
        {
            moveL(Taihu, pos2, 0.2);
            return -1;
        }
    }
    else
    {

        ret1 = moveL(Taihu, goal_last, 0.2);

        if (ret1 == -1)
        {
            moveL(Taihu, pos1, 0.2);
            return -1;
        }
    }
    // {
    //     sleep(2);
    //     MatrixXd current_hand_pos = Taihu.getTcpPos();
    //     cout << "抓取函数当前位置" << current_hand_pos << endl;
    //     cout << "抓取函数目标位置" << goal_last[0] << " " << goal_last[1] << " " << goal_last[2] << endl;
    // }

    // if (flag == 1)
    // {
    //     tiger_hand(right_a);
    // }
    // else
    // {
    //     tiger_hand(left_a);
    // }
    if (flag == 1)
    {

        double x_qian = -readPiancha("1") + readPiancha("3");
        goal_last[0] += x_qian;
    }
    else
    {

        double x_qian = -readPiancha("1") + readPiancha("4");
        goal_last[0] += x_qian;
    }

    // cin >> stop;
    // cin.ignore(numeric_limits<streamsize>::max(), '\n');

    if (flag == 2) // left
    {

        ret1 = moveL(Taihu, goal_last, 0.1);

        if (ret1 == -1)
        {

            moveL(Taihu, pos2, 0.2);
            return -1;
        }
    }
    else
    {

        ret1 = moveL(Taihu, goal_last, 0.1);

        if (ret1 == -1)
        {
            moveL(Taihu, pos1, 0.2);

            return -1;
        }
    }
    // {
    //     sleep(2);
    //     MatrixXd current_hand_pos = Taihu.getTcpPos();
    //     cout << "抓取函数last当前位置" << current_hand_pos << endl;
    //     cout << "抓取函数last目标位置" << goal_last[0] << " " << goal_last[1] << " " << goal_last[2] << endl;
    // }

    // sleep(1);

    if (flag == 1)
    {
        if (target == "liziyuan" || target == "green")
        {
            cout << "target == liziyuan || target == green" << endl;
            grasp_hand2(right_a);
        }
        else
        {
            grasp_hand(right_a);
            // sleep(1);
        }
    }
    else
    {
        if (target == "liziyuan" || target == "green")
        {
            cout << "target == liziyuan || target == green" << endl;
            grasp_hand2(left_a);
        }
        else
        {
            grasp_hand(left_a);
            // sleep(1);
        }
    }

    // cin >> stop;
    // cin.ignore(numeric_limits<streamsize>::max(), '\n');

    return 0;
}

int convert_postion(Matrix<double, 1, 6> &goal_last, std::array<double, 3> coords, int flag)
{
    // float rotation = 0 * M_PI / 180;
    // Eigen::Matrix3d R = Roll_Rotation(rotation);
    Eigen::Vector3d t;
    t << coords[0] * 1000, coords[1] * 1000, coords[2] * 1000;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    // T.block<3, 3>(0, 0) = R; // Set rotation
    T.block<3, 1>(0, 3) = t; // Set translation

    // cout << "-----------------------------------------------------------" << endl;

    Eigen::Vector3d goal_t;
    Eigen::Matrix4d robot_t = neck_to_eye(flag) * T;
    // cout << "robot_t: " << endl << robot_t << endl;
    goal_t << robot_t.block<3, 1>(0, 3);
    // cout << goal_t(0) << "," << goal_t(1) << "," << goal_t(2) << endl;
    goal_last << goal_t(0) / 1000, goal_t(1) / 1000, goal_t(2) / 1000, 0, 0, 0;

    return 0;
}

std::string read_server_response()
{
    boost::asio::streambuf response;
    boost::asio::read_until(*g_socket, response, '\n');
    std::istream is(&response);
    std::string result;
    std::getline(is, result);
    return result;
}

bool reals_tcp_init2(const std::string &host, int port)
{
    try
    {
        static boost::asio::io_service io_service;
        g_socket = new tcp::socket(io_service);
        g_socket->connect(tcp::endpoint(boost::asio::ip::address::from_string(host), port));

        // 关键修复：连接后立即读取并丢弃欢迎消息
        std::string welcome_msg = read_server_response();
        std::cout << "服务器问候: " << welcome_msg << std::endl;

        return true;
    }
    catch (...)
    {
        cout << "相机链接失败" << endl;
        return false;
    }
}

int reals_tcp_send_command(const std::string &command)
{
    try
    {
        boost::asio::write(*g_socket, boost::asio::buffer(command + "\n"));
        return 0; // 使用统一响应读取函数
    }
    catch (...)
    {
        return -1;
    }
}

void reals_tcp_close()
{
    if (g_socket)
    {
        try
        {
            if (g_socket->is_open())
            {
                g_socket->close();
            }
            delete g_socket;
            g_socket = nullptr;
            std::cout << "✅ 已断开与Python服务器的连接" << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "❌ 关闭连接时出错: " << e.what() << std::endl;
        }
    }
}

bool extractCoordinates(const std::string &input, double coordinates[3])
{
    // 找到括号的位置
    size_t start = input.find('(');
    size_t end = input.find(')');

    if (start == std::string::npos || end == std::string::npos)
    {
        std::cerr << "错误：无法找到坐标数据" << std::endl;
        return false;
    }

    // 提取括号内的内容
    std::string coords_str = input.substr(start + 1, end - start - 1);

    // 使用stringstream分割字符串
    std::stringstream ss(coords_str);
    char comma;

    // 直接读取到数组中
    if (!(ss >> coordinates[0] >> comma >> coordinates[1] >> comma >> coordinates[2]))
    {
        std::cerr << "错误：坐标格式不正确" << std::endl;
        return false;
    }

    return true;
}

int extractSegmentNumber(const std::string &input)
{
    const std::string key = "分段编号: ";
    size_t pos = input.find(key); // 查找 "分段编号: " 的位置

    if (pos == std::string::npos)
    {
        std::cerr << "Error: '分段编号' not found in the input string." << std::endl;
        return -1; // 返回错误码
    }

    pos += key.length(); // 跳过 "分段编号: "，定位到数字开始位置
    int segmentNumber = 0;

    // 提取数字部分
    while (pos < input.size() && isdigit(input[pos]))
    {
        segmentNumber = segmentNumber * 10 + (input[pos] - '0');
        pos++;
    }

    return segmentNumber;
}

#include "wt_client.h"

void hand_mode(Robot_Arm &Taihu, int hand_flag, int sock, string target)
{
    // 注意：必须严格使用小写命令

    // string hand_move = "human_hand";

    Matrix<double, 1, 6> human_hand_position;
    // std::vector<std::string> req_right;
    // req_right.reserve(11);
    // req_right.push_back("0");  // alg_witch
    // req_right.push_back("0");  // head_roll
    // req_right.push_back("30"); // head_pitch
    // req_right.push_back("0");  // head_yaw
    // for (int i = 0; i < 6; ++i)
    //     req_right.push_back("0");   // 6 个预留
    // req_right.push_back(hand_move); // 类别_id
    // int error = 0;
    // while (1)
    // {
    //     if (error > 11)
    //     {
    //         break;
    //     }
    //     int get_drink_posi = get_postion(sock, req_right, human_hand_position);
    //     if (get_drink_posi != -1)
    //     {
    //         error=12;
    //         break;
    //     }
    //     ++error;
    // }

    // if (error > 11)
    // {
    if (hand_flag == 1)
    {
        human_hand_position << 0.55, -0.2, -0.21, 0, 0, 0;

        cout << "测试555" << endl;

        human_hand_position[0] = readPiancha("6000");
        human_hand_position[1] = readPiancha("6001");
        human_hand_position[2] = readPiancha("6002");

        moveL(Taihu, human_hand_position, 0.2);
        return;
    }
    else
    {
        human_hand_position << 0.55, 0.2, -0.21, 0, 0, 0;
        human_hand_position[0] = readPiancha("6003");
        human_hand_position[1] = readPiancha("6004");
        human_hand_position[2] = readPiancha("6005");
        moveL(Taihu, human_hand_position, 0.2);
        return;
    }
    // }
}

int moveL(Robot_Arm &Taihu, Matrix<double, 1, 6> posd, double speed)
{
    usleep(250000);
    return arm_line_move(Taihu, posd, speed);
}

string jianlue(string target)
{
    if (target == "mnd" || target == "meinianda")
    {
        return "meinianda";
    }
    else if (target == "xb" || target == "xuebi")
    {
        return "xuebi";
    }
    else if (target == "yq" || target == "yiquan")
    {
        return "yiquan";
    }
    else if (target == "cc" || target == "coca")
    {
        return "coca";
    }
    else if (target == "bskl" || target == "baishikele")
    {
        return "baishikele";
    }
    else if (target == "jlb" || target == "jianlibao")
    {
        return "jianlibao";
    }
    else if (target == "mz" || target == "mozhua")
    {
        return "mozhua";
    }
    else if (target == "wtkl" || target == "wutangkele")
    {
        return "wutangkele";
    }
    else if (target == "md" || target == "maidong")
    {
        return "maidong";
    }
    else if (target == "wlc" || target == "wulongcha")
    {
        return "wulongcha";
    }
    else if (target == "yq" || target == "yuanqi")
    {
        return "yuanqi";
    }
    else if (target == "mzy" || target == "meizhiyuan")
    {
        return "meizhiyuan";
    }
    else if (target == "bhc" || target == "binghongcha")
    {
        return "binghongcha";
    }
    else if (target == "yykx" || target == "yingyangkuaixian")
    {
        return "yingyangkuaixian";
    }
    else if (target == "bkl" || target == "baokuangli")
    {
        return "baokuangli";
    }
    else if (target == "dfsy" || target == "dongfangshuye")
    {
        return "dongfangshuye";
    }
    else
    {
        return "no_drink_name"; // or return target; depending on your needs
    }
}

// 读取偏差文件并返回指定位置的数值（每次调用完整重读文件，便于不停机改参；依赖进程当前工作目录下的 piancha.txt）
double readPiancha(const std::string &identifier)
{
    std::ifstream file("piancha.txt");
    std::map<std::string, double> dataMap; // 使用map存储标识符和数值的映射

    if (file.is_open())
    {
        std::string line;
        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string item;

            // 以逗号为分隔符读取每个条目
            while (std::getline(ss, item, ','))
            {
                // 去除首尾空格
                item.erase(0, item.find_first_not_of(" \t"));
                item.erase(item.find_last_not_of(" \t") + 1);

                // 查找冒号位置
                size_t colonPos = item.find(':');
                if (colonPos != std::string::npos)
                {
                    std::string key = item.substr(0, colonPos);
                    std::string valueStr = item.substr(colonPos + 1);

                    try
                    {
                        double value = std::stod(valueStr);
                        // 同时存储完整标识符和数字部分
                        dataMap[key] = value;

                        // 如果标识符以数字开头，也存储数字部分
                        size_t underscorePos = key.find('_');
                        if (underscorePos != std::string::npos)
                        {
                            std::string numPart = key.substr(0, underscorePos);
                            dataMap[numPart] = value;
                        }
                    }
                    catch (...)
                    {
                        continue; // 跳过无效数值
                    }
                }
            }
        }
        file.close();

        // 查找匹配的标识符
        auto it = dataMap.find(identifier);
        if (it != dataMap.end())
        {
            return it->second;
        }
        else
        {
            std::cerr << "错误：未找到标识符 '" << identifier << "'" << std::endl;
            return 0.0;
        }
    }
    else
    {
        std::cerr << "错误：无法打开文件 piancha.txt" << std::endl;
        return 0.0;
    }
}

void change_pian(string traget, Matrix<double, 1, 6> &goal_last, double lin, aoyi_hand ti5_hand, int drink_layer)
{
    goal_last[2] = -0.165;

    if (drink_layer == 1)
    {
        goal_last[2] = readPiancha("30");
    }
    else if (drink_layer == 2)
    {
        goal_last[2] = readPiancha("31");
    }
    else if (drink_layer == 3)
    {
        goal_last[2] = readPiancha("32");
    }

    if (traget == "meinianda" || traget == "xuebi" || traget == "yiquan" || traget == "coca" || traget == "baishikele" || traget == "jianlibao" || traget == "mozhua" || traget == "wutangkele")
    {
        if (lin > 0.59) // 第二排
        {
            if (ti5_hand == right_a)
            {

                goal_last[1] += readPiancha("13");
                goal_last[0] += readPiancha("9");
            }
            else
            {

                goal_last[1] += readPiancha("15");
                goal_last[0] += readPiancha("11");
            }
        }
        else
        {
            if (ti5_hand == right_a)
            {

                goal_last[1] += readPiancha("12");
                ;
                goal_last[0] += readPiancha("8");
            }
            else
            {

                goal_last[1] += readPiancha("14");
                goal_last[0] += readPiancha("10");
            }
        }
    }
    else
    {
        if (lin < 0.59)
        {
            if (ti5_hand == right_a)
            {

                goal_last[1] += readPiancha("20");
                goal_last[0] += readPiancha("16");
            }
            else
            {

                goal_last[1] += readPiancha("22");
                goal_last[0] += readPiancha("18");
            }
        }
        else
        {
            if (ti5_hand == right_a)
            {

                goal_last[1] += readPiancha("21");
                goal_last[0] += readPiancha("17");
            }
            else
            {

                goal_last[1] += readPiancha("23");
                goal_last[0] += readPiancha("19");
            }
        }
    }
}

int convert_biaoding(Matrix<double, 1, 6> &goal_last, std::array<double, 3> coords)
{

    //     euler_angles_xyz: [-134.1044, 1.6954, -88.71701]
    // matrix: [[0.02238, -0.69627, 0.71743, 98.13279], [-0.99931, 0.00566, 0.03666, 37.77919],
    //   [-0.02959, -0.71776, -0.69566, 180.15813], [0.0, 0.0, 0.0, 1.0]]
    Matrix<double, 4, 4> T_base_eye;
    Matrix<double, 4, 1> T_posd;
    Matrix<double, 4, 1> posd;
    T_base_eye << 0.02238, -0.69627, 0.71743, 98.13279 / 1000,
        -0.99931, 0.00566, 0.03666, 37.77919 / 1000,
        -0.02959, -0.71776, -0.69566, 180.15813 / 1000,
        0.0, 0.0, 0.0, 1.0;
    T_posd << coords[0], coords[1], coords[2], 1;

    posd = T_base_eye * T_posd;
    // cout << "posd: " << posd <<endl;
    // cout << "coords: "  <<coords[0]<< " "<< coords[1]<<" " <<coords[2]<<" " <<endl;
    // cout <<"T_base_eye" <<T_base_eye << endl;
    // cout << "T_posd" <<T_posd <<endl;
    for (int i = 0; i < 3; i++)
    {
        goal_last[i] = posd(i);
    }
    goal_last[3] = 0;
    goal_last[4] = 0;
    goal_last[5] = 0;
    return 0;
}
