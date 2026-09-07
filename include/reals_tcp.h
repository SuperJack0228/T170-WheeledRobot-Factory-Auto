#ifndef REALS_TCP_H
#define REALS_TCP_H

#include <iostream>
#include <boost/asio.hpp>
#include <Eigen/Dense>
#include "function.h"

using boost::asio::ip::tcp;

tcp::socket reals_tcp_init();



extern tcp::socket *g_socket ;



std::string reals_tcp_trsmit(tcp::socket &socket, std::string target);



//-------------------------------------------------------------------------------------------------------//

int pos_move(double goal_last[3], Robot_Arm &Taihu);


int moveL(Robot_Arm &Taihu, Matrix<double, 1, 6> posd, double speed);


int position_con(Robot_Arm &Taihu);

int start_pos(Robot_Arm &Taihu_r, Robot_Arm &Taihu_l, int flag = 2);


int zhuaqu(Matrix<double, 1, 6> &goal_last, Robot_Arm &Taihu, int flag, std::string target, double lin);

int convert_postion(Matrix<double, 1, 6> &goal_last,std::array<double, 3> coords,int flag=0);



//-------------------------------------------------------------------------------------------------------//


std::string read_server_response();


bool reals_tcp_init2(const std::string &host = "127.0.0.1", int port = 12345);


int reals_tcp_send_command(const std::string &command);


void reals_tcp_close();


void hand_mode(Robot_Arm &Taihu, int hand_flag, int sock = 1000, string target = "");


bool extractCoordinates(const std::string &input, double coordinates[3]);

int extractSegmentNumber(const std::string &input);



int mech(Robot_Arm &Taihu, Robot_Arm &Taihu_l);


string jianlue(string target);

// 每次调用都会重新打开并解析当前工作目录下的 piancha.txt（无缓存）；保存后下次 readPiancha("键") 即读到新值。
double readPiancha(const std::string& identifier);

void change_pian(string traget, Matrix<double, 1, 6> &goal_last,double lin,aoyi_hand ti5_hand,int drink_layer=1);

int convert_biaoding(Matrix<double, 1, 6> &goal_last, std::array<double, 3> coords);





#endif