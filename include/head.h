#ifndef HEAD_H
#define HEAD_H

#include <Eigen/Dense>
#include <vector>
#include <stdio.h>
#include <cstring>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <csignal>
#include <mutex>
#include <shared_mutex>
#include <map>
#include <termios.h>
#include <fcntl.h>
#include <algorithm>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <thread>
#include <array>
#include <boost/asio.hpp>

#include "function.h"
#include "controlcan.h"
#include "aoyihand.h"

inline constexpr double deg2rad = M_PI / 180.0;
inline constexpr double rad2deg = 180.0 / M_PI;
inline constexpr double rad = M_PI / 180.0;
inline constexpr double deg = 180.0 / M_PI;
inline constexpr double eps = 1e-6;



inline int stop;

#endif