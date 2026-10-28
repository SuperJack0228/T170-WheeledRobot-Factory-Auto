
#include <iostream>
#include <fstream>
#include <iomanip>

#include "function.h"

#include "Ti5_socketcan.h"
#include "hand_shadow.h"
#include "realsense_get.h"

#include <atomic>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cmath>
#include <functional>
#include <sstream>
#include <string>
#include <filesystem>
#include <mutex>
#include <vector>

#include "move_box_config.h"
#include "seg_pose_bridge.h"
// 将沿着xyz坐标系平移运动转化为4*4矩阵
// #define pi acos(-1)
// const double eps = 1e-6;
// double rad=pi/180,deg=180/pi;
//  double rad()
//  {
//      return pi/180;
//  }
//  double deg()
//  {
//      return 180/pi;
//  }
// 将沿着xyz坐标系平移运动转化为4*4矩阵
MatrixXd Transl_xyz(Matrix<double, 1, 3> trans)
{
    double x = trans(0), y = trans(1), z = trans(2);
    Matrix<double, 4, 4> T_transl;
    T_transl << 1, 0, 0, x,
        0, 1, 0, y,
        0, 0, 1, z,
        0, 0, 0, 1;
    return T_transl;
}

// 将沿着xyz坐标系旋转运动转化为4*4矩阵
// 将沿着xyz坐标系旋转运动转化为4*4矩阵
MatrixXd Rot_zyx(Matrix<double, 1, 3> theta)
{
    Matrix<double, 4, 4> T_R;
    Matrix<double, 3, 3> Rotx, Roty, Rotz, RPY;
    double a = theta(0), b = theta(1), c = theta(2);
    Rotz(0, 0) = cos(c);
    Rotz(0, 1) = -sin(c);
    Rotz(0, 2) = 0;
    Rotz(1, 0) = sin(c);
    Rotz(1, 1) = cos(c);
    Rotz(1, 2) = 0;
    Rotz(2, 0) = 0;
    Rotz(2, 1) = 0;
    Rotz(2, 2) = 1;

    Roty(0, 0) = cos(b);
    Roty(0, 1) = 0;
    Roty(0, 2) = sin(b);
    Roty(1, 0) = 0;
    Roty(1, 1) = 1;
    Roty(1, 2) = 0;
    Roty(2, 0) = -sin(b);
    Roty(2, 1) = 0;
    Roty(2, 2) = cos(b);

    Rotx(0, 0) = 1;
    Rotx(0, 1) = 0;
    Rotx(0, 2) = 0;
    Rotx(1, 0) = 0;
    Rotx(1, 1) = cos(a);
    Rotx(1, 2) = -sin(a);
    Rotx(2, 0) = 0;
    Rotx(2, 1) = sin(a);
    Rotx(2, 2) = cos(a);

    RPY = Rotz * Roty * Rotx;
    T_R.row(0) << RPY.row(0), 0;
    T_R.row(1) << RPY.row(1), 0;
    T_R.row(2) << RPY.row(2), 0;
    T_R.row(3) << 0, 0, 0, 1;
    return T_R;
}

MatrixXd TR(Matrix<double, 1, 6> transpose)
{

    Matrix<double, 1, 3> transl, rot;
    Matrix<double, 4, 4> T;
    transl << transpose(0), transpose(1), transpose(2);
    rot << transpose(3), transpose(4), transpose(5);
    T = Transl_xyz(transl) * Rot_zyx(rot);
    return T;
}

// 修正DH坐标
MatrixXd MDHTrans(double alpha, double a, double d, double theta)
{
    MatrixXd T(4, 4);
    T(0, 0) = cos(theta);
    T(0, 1) = -sin(theta);
    T(0, 2) = 0;
    T(0, 3) = a;
    T(1, 0) = sin(theta) * cos(alpha);
    T(1, 1) = cos(theta) * cos(alpha);
    T(1, 2) = -sin(alpha);
    T(1, 3) = -sin(alpha) * d;
    T(2, 0) = sin(theta) * sin(alpha);
    T(2, 1) = cos(theta) * sin(alpha);
    T(2, 2) = cos(alpha);
    T(2, 3) = cos(alpha) * d;
    T(3, 0) = 0;
    T(3, 1) = 0;
    T(3, 2) = 0;
    T(3, 3) = 1;

    return T;
}
Matrix<double, 1, 3> rotationMatrixToEulerAngles(Matrix<double, 3, 3> R)
{
    //    assert(isRotationMatrix(R));
    double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular)
    {
        x = atan2(R(2, 1), R(2, 2));
        y = atan2(-R(2, 0), sy);
        z = atan2(R(1, 0), R(0, 0));
    }
    else
    {
        x = atan2(-R(1, 2), R(1, 1));
        y = atan2(-R(2, 0), sy);
        z = 0;
    }
    return {x, y, z};
}

Matrix<double, 1, 6> T2PosEulerAngles(Matrix<double, 4, 4> T)
{
    Matrix<double, 1, 3> p, euler;
    Matrix<double, 1, 6> pos;
    euler = rotationMatrixToEulerAngles(T.block(0, 0, 3, 3));
    p << T(0, 3), T(1, 3), T(2, 3);
    pos << p, euler;
    return pos;
}

Eigen::Vector3d Quaterniond2EulerAngles(Eigen::Quaterniond q)
{
    Eigen::Vector3d angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles(0) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        angles(1) = std::copysign(M_PI / 2, sinp); // use 90 deg()rees if out of range
    else
        angles(1) = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    angles(2) = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}
// 4*4位姿矩阵转化为1*3轴角
// MatrixXd T_to_AngleAxis(Matrix<double, 4, 4>  T)
//{
//     Matrix<double, 4, 4> Tcp;
//     Matrix<double, 1, 3> vec,AngleAxis;
//     double k,theta;
//     double r11,r12,r13,r21,r22,r23,r31,r32,r33;
//     r11 = T(0, 0); r12 = T(0, 1); r13 = T(0, 2);
//     r21 = T(1, 0); r22 = T(1, 1); r23 = T(1, 2);
//     r31 = T(2, 0); r32 = T(2, 1); r33 = T(2, 2);
//     theta=acos((r11+r22+r33-1)/2);
//     k=(1/(2*sin(theta)));
//     vec<<r32-r23,r13-r31,r21-r12;
//     AngleAxis=theta*k*vec;
//     return AngleAxis;
// }

MatrixXd T2Axispos(Matrix<double, 4, 4> T)
{

    Matrix<double, 3, 3> rotation_matrix;
    Matrix<double, 3, 1> axis;
    Matrix<double, 1, 6> pos;
    rotation_matrix = T.block(0, 0, 3, 3);
    Eigen::AngleAxisd angel_axisd(rotation_matrix);
    axis << angel_axisd.axis() * angel_axisd.angle();
    //      cout<<"axis="<<angel_axisd.axis()*angel_axisd.angle()<<endl;
    if (axis(1) > M_PI)
    {
        axis(1) = axis(1) - 2 * M_PI;
    }
    else if (axis(1) < -M_PI)
    {
        axis(1) = axis(1) + 2 * M_PI;
    }
    pos << T(0, 3), T(1, 3), T(2, 3), axis.transpose();
    return pos;
}
// 4*4位姿矩阵转化为1*6 Pos即位置xyz加轴角
Eigen::VectorXd T2Axis(const Eigen::Matrix4d &T)
{
    // 提取旋转矩阵元素
    double r11 = T(0, 0);
    double r12 = T(0, 1);
    double r13 = T(0, 2);
    double r21 = T(1, 0);
    double r22 = T(1, 1);
    double r23 = T(1, 2);
    double r31 = T(2, 0);
    double r32 = T(2, 1);
    double r33 = T(2, 2);

    // 计算旋转角度 theta
    //        cout<<"r11 + r22 + r33 - 1"<<","<<(r11 + r22 + r33 - 1) / 2<<endl;
    double theta;
    //        cout<<"T"<<T<<""<<","<<abs((r11 + r22 + r33 - 1) / 2)<<endl;
    Eigen::Vector3d angle_axis;
    if (std::abs(abs((r11 + r22 + r33 - 1) / 2) - 1) < eps)
    {
        // 当 theta 接近 0 时，轴角为零向量
        angle_axis.setZero();
    }
    else
    {
        theta = std::acos((r11 + r22 + r33 - 1) / 2);
        // 计算旋转轴 v
        Eigen::Vector3d v;
        v(0) = (r32 - r23) / (2 * std::sin(theta));
        v(1) = (r13 - r31) / (2 * std::sin(theta));
        v(2) = (r21 - r12) / (2 * std::sin(theta));

        // 计算轴角表示
        angle_axis = theta * v;
    }

    // 提取位置信息
    double x = T(0, 3);
    double y = T(1, 3);
    double z = T(2, 3);

    // 组合位置和轴角信息到 Pos 向量
    Eigen::VectorXd Pos(6);
    Pos << x, y, z, angle_axis(0), angle_axis(1), angle_axis(2);

    return Pos;
}
MatrixXd Axispos2T(Matrix<double, 1, 6> pos)
{
    Matrix<double, 4, 4> T;
    Matrix<double, 3, 1> vec;
    vec << pos(3), pos(4), pos(5);

    //    Eigen::AngleAxisd angel_axisd(rotation_matrix);
    Eigen::AngleAxisd rot_vec(vec.norm(), vec.normalized());
    Eigen::Matrix3d M = rot_vec.toRotationMatrix();
    T.block(0, 3, 3, 1) << pos(0), pos(1), pos(2);
    T.block(0, 0, 3, 3) << M;
    T.block(3, 0, 1, 4) << 0, 0, 0, 1;
    if (pos(3) == 0 && pos(4) == 0 && pos(5) == 0)
    {
        T.block(0, 0, 3, 3) << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    }
    return T;
}

// MatrixXd ndi2T(Matrix<double, 1, 6>  pos)
// {
//     Matrix<double, 1, 6> transpose;
//     Matrix<double, 4, 4> T;
//     double x=pos(0)*pow(10,-3),y=pos(1)*pow(10,-3),z=pos(2)*pow(10,-3),
//         rx=pos(3)*rad(),ry=pos(4)*rad(),rz=pos(5)*rad();
//     transpose<<x,y,z,rx,ry,rz;
//     T=TR(transpose);
//     return T;
// }

// MatrixXd displaypos(Matrix<double, 1, 6>  pos)
// {
//     Matrix<double, 1, 6> displaypos;
//     displaypos<<pos(0)*pow(10,3),pos(1)*pow(10,3),pos(2)*pow(10,3),
//         pos(3)*deg(),pos(4)*deg(),pos(5)*deg();
//     return displaypos;
// }
MatrixXd skew(Matrix<double, 1, 3> s)

{
    double vx = s(0), vy = s(1), vz = s(2);
    Matrix<double, 3, 3> R;
    R << 0, -vz, vy,
        vz, 0, -vx,
        -vy, vx, 0;
    return R;
}
MatrixXd line_interp(Matrix<double, 1, 6> p1, Matrix<double, 1, 6> p2, double dx)

{

    Matrix<double, 1, 6> dp, d, a, alp, offset;
    int m, n, j;
    double len, k;

    dp = p1 - p2;
    len = sqrt(pow(dp(0), 2) + pow(dp(1), 2) + pow(dp(2), 2));
    n = len / dx;
    MatrixXd Pos(n + 2, 6);
    m = n + 1;
    j = 1;
    for (int i = 0; i <= m; i++)
    {
        k = i / double(m);
        Pos.block(i, 0, 1, 6) = p1 + k * (p2 - p1);
    }
    return Pos;
}

MatrixXd circular_interp(Matrix<double, 1, 6> p1, Matrix<double, 1, 6> p2, Matrix<double, 1, 6> p3, double dx)

{

    Matrix<double, 1, 6> o13;
    int m, n, j;
    double len, k, x1, y1, z1, x2, y2, z2, x3, y3, z3, A1, B1, C1, D1, A2, B2, C2, D2, A3, B3, C3, D3, x0, y0, z0, r,
        ax, ay, az, L, d13, do2, theta, q;
    Matrix<double, 3, 3> A;
    Matrix<double, 3, 1> b, C, u, v, p;

    x1 = p1(0);
    x2 = p2(0);
    x3 = p3(0);
    y1 = p1(1);
    y2 = p2(1);
    y3 = p3(1);
    z1 = p1(2);
    z2 = p2(2);
    z3 = p3(2);

    A1 = (y1 - y3) * (z2 - z3) - (y2 - y3) * (z1 - z3);
    B1 = (x2 - x3) * (z1 - z3) - (x1 - x3) * (z2 - z3);
    C1 = (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3);
    D1 = -(A1 * x3 + B1 * y3 + C1 * z3);

    A2 = x2 - x1;
    B2 = y2 - y1;
    C2 = z2 - z1;
    D2 = -((pow(x2, 2) - pow(x1, 2)) + (pow(y2, 2) - pow(y1, 2)) + (pow(z2, 2) - pow(z1, 2))) / 2;

    A3 = x3 - x2;
    B3 = y3 - y2;
    C3 = z3 - z2;
    D3 = -((pow(x3, 2) - pow(x2, 2)) + (pow(y3, 2) - pow(y2, 2)) + (pow(z3, 2) - pow(z2, 2))) / 2;

    A << A1, B1, C1,
        A2, B2, C2,
        A3, B3, C3;
    b << -D1, -D2, -D3;

    //% 圆心
    C = A.inverse() * b;
    x0 = C(0);
    y0 = C(1);
    z0 = C(2);

    //% 外接圆半径
    r = sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2) + pow(z1 - z0, 2));

    //% 新坐标系Z0的方向余弦
    L = sqrt(pow(A1, 2) + pow(B1, 2) + pow(C1, 2));
    ax = A1 / L;
    ay = B1 / L;
    az = C1 / L;
    //% 新坐标系X0的方向余弦

    //%step2:得到13两点间的间距d
    d13 = sqrt(pow(p1(0) - p3(0), 2) + pow(p1(1) - p3(1), 2) + pow(p1(2) - p3(2), 2));
    o13 = (p1 + p3) / 2;

    do2 = sqrt(pow(o13(0) - p2(0), 2) + pow(o13(1) - p2(1), 2) + pow(o13(2) - p2(2), 2));
    // d13=sqrt((p1(1)-p3(1))^2+(p1(2)-p3(2))^2+(p1(3)-p3(3))^2);
    // o13=(p1+p3)/2;%得到13两点的中点，当弧角180度时，圆心在该点，以该点为衡量，判定优劣弧
    //%step3:得到13中点与2点间距do2
    // do2=sqrt((o13(1)-p2(1))^2+(o13(2)-p2(2))^2+(o13(3)-p2(3))^2);
    //%ste4:判断优弧还是劣弧
    if (do2 < d13 / 2)
    {
        theta = 2 * asin(d13 / 2 / r);
    }
    else
    {
        theta = 2 * M_PI - 2 * asin(d13 / 2 / r);
    }

    len = theta * r;
    //% 插补点数N
    n = len / dx;
    MatrixXd Pos(n + 2, 6);
    m = n + 1;
    v = (p1.block(0, 0, 1, 3).transpose() - C);
    u << ax, ay, az;
    j = 1;

    for (int i = 0; i <= m; i++)
    {
        k = i / double(m);
        q = theta * k;
        p = v * cos(q) + u.dot(v) * u * (1 - cos(q)) + u.cross(v) * sin(q) + C;

        Pos.block(i, 0, 1, 3) = p.transpose();
        Pos.block(i, 3, 1, 3) = p1.block(0, 3, 1, 3);
    }
    return Pos;
}

Eigen::MatrixXd pinv(Eigen::MatrixXd A) // 计算矩阵伪逆
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double pinvtoler = 1.e-8; // tolerance
    int row = A.rows();
    int col = A.cols();
    int k = min(row, col);
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col, row);
    Eigen::MatrixXd singularValues_inv = svd.singularValues(); // 奇异值
    Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
    for (long i = 0; i < k; ++i)
    {
        if (singularValues_inv(i) > pinvtoler)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else
            singularValues_inv(i) = 0;
    }
    for (long i = 0; i < k; ++i)
    {
        singularValues_inv_mat(i, i) = singularValues_inv(i);
    }
    X = (svd.matrixV()) * (singularValues_inv_mat) * (svd.matrixU().transpose());

    return X;
}

// 正运动学 根据输入关节角度求出末端法兰相对于基座4*4矩阵
MatrixXd fkine(Matrix<double, 1, 6> theta)
{

    Matrix<double, 1, 6> d, a, alp, offset;
    Matrix<double, 4, 4> T1, T2, T3, T4, T5, T6, T7, T;
    Matrix<double, 1, 6> p;

    //    alp << 0,90,180,90,-90,90;
    //    a << 0,0,0.480015,0,0,0;
    //    d <<0.26,0,0,0.520137,0,0.192;
    //    offset<<0,90,90,0,0,0;
    //    alp=alp*rad();
    //    offset=offset*rad();

    //    alp<<0,-1.57104,-0.000294356,1.57027,-1.57031,1.56901;
    //    a<<   0,0.000175529,0.419961,0.000144852,-1.9803e-05,-0.069077;
    //    d<< 0.2665,-0.000116339,0,0.380242,-0.000232702,0.0951;
    //    offset<< 0,-1.56438,1.57089,0.00632152,-0.00257079,0;
    alp << 0, -1.57109, -0.000382754, 1.5709, -1.57184, 1.57136;
    a << 0, -3.37469e-05, 0.469603, -2.73644e-05, -0.000133765, -0.0776986;
    d << 0.326, -7.60105e-05, 0, 0.429973, 9.031e-05, 0.0973;
    offset << 0, -1.57001, 1.56756, -0.00575697, -0.00305482, 0;
    theta = theta + offset;
    T1 = MDHTrans(alp(0), a(0), d(0), theta(0));
    T2 = MDHTrans(alp(1), a(1), d(1), theta(1));
    T3 = MDHTrans(alp(2), a(2), d(2), theta(2));
    T4 = MDHTrans(alp(3), a(3), d(3), theta(3));
    T5 = MDHTrans(alp(4), a(4), d(4), theta(4));
    T6 = MDHTrans(alp(5), a(5), d(5), theta(5));
    //    T7 = MDHTrans(alp(6), a(6), d(6), theta(6));
    T = T1 * T2 * T3 * T4 * T5 * T6;
    // p = T_to_RPY(T);

    //    cout<<T<<endl;
    return T;
}

Eigen::Matrix<double, 1, 6> quinticInterp_CartSpace(
    double t,
    double totalTime,
    const Eigen::Matrix<double, 1, 6> &start,
    const Eigen::Matrix<double, 1, 6> &target)
{
    // 处理总时间为0的特殊情况
    if (totalTime <= 1e-6)
        return target;

    // 时间边界处理（C++14兼容）
    double clampedTime = t;
    if (clampedTime < 0.0)
        clampedTime = 0.0;
    if (clampedTime > totalTime)
        clampedTime = totalTime;
    const double ratio = clampedTime / totalTime;

    // 五次多项式核心计算
    const double r2 = ratio * ratio;
    const double polyTerm = r2 * ratio * (10 - 15 * ratio + 6 * r2);

    // 计算当前位置
    return start + (target - start) * polyTerm;
}

Eigen::Matrix<double, 1, 7> quinticInterp_JointSpace(
    double t,
    double totalTime,
    const Eigen::Matrix<double, 1, 7> &start,
    const Eigen::Matrix<double, 1, 7> &target)
{
    // 处理总时间为0的特殊情况
    if (totalTime <= 1e-6)
        return target;

    // 时间边界处理（C++14兼容）
    double clampedTime = t;
    if (clampedTime < 0.0)
        clampedTime = 0.0;
    if (clampedTime > totalTime)
        clampedTime = totalTime;
    const double ratio = clampedTime / totalTime;

    // 五次多项式核心计算
    const double r2 = ratio * ratio;
    const double polyTerm = r2 * ratio * (10 - 15 * ratio + 6 * r2);

    // 计算当前位置
    return start + (target - start) * polyTerm;
}
// Matrix<double, 4, 4>  eyehand_falan_calib(Matrix<double, 8, 6> Joint, Matrix<double, 8, 6> NDIpose)
// {
//     int len=8,nn=len-1;
//     MatrixXd T_base_tool(4,4*len),T_cam_cal(4,4*len);
//     MatrixXd M(4*len,4),N(4*len,4),sum_T_tool_cal(4,4);
//     sum_T_tool_cal.setZero(4,4);
//     Matrix<double, 3, 3>  A1,B1;
//     MatrixXd A(4,4*nn),B(4,4*nn);
//     MatrixXd AA(4*nn,4);
//     Matrix<double, 1, 6> joint;
//     Matrix<double, 1, 6> ndi_pos;

//     for (int i = 0; i < len;i++)
//     {
//         joint<<Joint.block(i,0,1,6);
//         T_base_tool.block(0,4*i,4,4) =fkine(joint);
//         ndi_pos<<NDIpose.block(i,0,1,6);
//         T_cam_cal.block(0,4*i,4,4)=ndi2T(ndi_pos);
//     }

//     for  (int i=0;i<len-1;i++)
//     {
//         int j=i+1;
//         A.block(0,4*i,4,4) =T_base_tool.block(0,4*i,4,4).inverse()*T_base_tool.block(0,4*j,4,4);
//         B.block(0,4*i,4,4) =T_cam_cal.block(0,4*i,4,4).inverse()*T_cam_cal.block(0,4*j,4,4);

//     }

//     for  (int i=0;i<len-1;i++)
//     {
//         A1=A.block(0,4*i,3,3);
//         B1=B.block(0,4*i,3,3);
//         Eigen::Quaterniond a(A1);
//         Eigen::Quaterniond b(B1);
//         Matrix<double, 1, 3>  m1,m2;
//         Matrix<double, 1, 4>  aq;
//         Matrix<double, 3, 3>  C1;
//         Matrix<double, 4, 4> AA1,BB1,dd;
//         m1<<a.x(),a.y(),a.z();
//         m2<<b.x(),b.y(),b.z();
//         aq<<a.x(),a.y(),a.z(),a.w();
//         MatrixXd::Identity(3,3) ;
//         C1.setIdentity(3,3) ;
//         AA1.block(0,0,1,4)<<a.w(),-a.x(),-a.y(),-a.z();
//         AA1.block(1,0,3,1)<<a.x(),a.y(),a.z();
//         AA1.block(1,1,3,3)=a.w()*C1+skew(m1);

//         BB1.block(0,0,1,4)<<b.w(),-b.x(),-b.y(),-b.z();
//         BB1.block(1,0,3,1)<<b.x(),b.y(),b.z();
//         BB1.block(1,1,3,3)=b.w()*C1-skew(m2);

//         AA.block(4*i,0,4,4)=AA1-BB1;

//     }

//     JacobiSVD<Eigen::MatrixXd> svd(AA,  ComputeThinU | ComputeThinV);
//     Matrix<double, 4, 4>T_tcp_calib, V = svd.matrixV();

//     Matrix<double, 4, 1> V1;
//     V1.block(0,0,3,1)=V.block(1,3,3,1);
//     V1(3,0)=V(0,3);
//     Eigen::Quaterniond v1q(V1);
//     Matrix<double, 3, 3> R=v1q.toRotationMatrix();

//     MatrixXd C(3*nn,3);
//     MatrixXd d(3*nn,1);
//     Matrix<double, 3, 3>  I;
//     I.setIdentity(3,3) ;
//     for  (int i=0;i<len-1;i++)
//     {
//         C.block(3*i,0,3,3)=I-A.block(0,4*i,3,3);
//         d.block(3*i,0,3,1)=A.block(0,4*i+3,3,1) -R*B.block(0,4*i+3,3,1);

//     }
//     Matrix<double, 3, 1>  t;
//     Matrix<double, 4, 4>  X1,t_tool_cal;
//     t=(C.transpose()*C).inverse()*C.transpose()*d;

//     T_tcp_calib.block(0,0,3,3)<<R;
//     T_tcp_calib.block(0,3,3,1)<<t;
//     T_tcp_calib.block(3,0,1,4)<<0,0,0,1;
//     return T_tcp_calib;
// }

// void Delay_mSec(unsigned int msec)
// {
//     QTime dieTime = QTime::currentTime().addMSecs(msec);
//     while (QTime::currentTime() < dieTime)
//         QCoreApplication::processEvents(QEventLoop::AllEvents, 5);
// }

// 六个自由度和六维度力对应相乘的函数
MatrixXd multiplyWithDOF(Matrix<double, 1, 6> dof, Matrix<double, 1, 6> forces)
{
    Matrix<double, 1, 6> result;
    for (int i = 0; i < 6; ++i)
    {
        result[i] = dof[i] * forces[i];
    }
    return result;
}

MatrixXd slerp_interp(double t, Matrix<double, 1, 3> euler_start, Matrix<double, 1, 3> euler_end)
{
    Matrix<double, 4, 4> T_start, T_end;
    Matrix<double, 3, 3> R_start, R_end;

    Matrix<double, 1, 3> RPY_interpolated, euler;
    Matrix<double, 1, 4> q1, q2, q_inter;
    double epsilon = 1e-6, omega;
    T_start = Rot_zyx(euler_start);
    T_end = Rot_zyx(euler_end);
    R_start = T_start.block(0, 0, 3, 3);
    R_end = T_end.block(0, 0, 3, 3);
    Eigen::Quaterniond q_start(R_start);
    Eigen::Quaterniond q_end(R_end);
    q1 << q_start.w(), q_start.x(), q_start.y(), q_start.z();
    q2 << q_end.w(), q_end.x(), q_end.y(), q_end.z();

    q_inter = q1;
    omega = acos(q1.dot(q2));
    if (abs(omega) < epsilon)
    {
        q_inter = (1 - t) * q1 + q2 * t;
    }
    else
    {
        q_inter = (sin((1 - t) * omega) / sin(omega)) * q1 + (sin(t * omega) / sin(omega)) * q2;
    }
    q_inter = q_inter / q_inter.norm();
    Eigen::Quaterniond interpolated_quaternions(q_inter(0), q_inter(1), q_inter(2), q_inter(3));
    euler = Quaterniond2EulerAngles(interpolated_quaternions);

    return euler;
}
//********************************

int getFileRows(const char *fileName)
{
    ifstream fileStream;
    string tmp;
    int count = 0;                      // 行数计数器
    fileStream.open(fileName, ios::in); // ios::in 表示以只读的方式读取文件
    if (fileStream.fail())              // 文件打开失败:返回0
    {
        return 0;
    }
    else // 文件存在
    {
        while (getline(fileStream, tmp, '\n')) // 读取一行
        {
            if (tmp.size() > 0)
                count++;
        }
        fileStream.close();
        return count;
    }
}

int getFileColumns(const char *fileName)
{
    ifstream fileStream;
    fileStream.open(fileName, ios::in);

    double tmp = 0;
    int count = 0; // 列数计数器
    char c;        // 当前位置的字符
    c = fileStream.peek();
    while (('\n' != c) && (!fileStream.eof())) // 指针指向的当前字符，仅观测，不移动指针位置
    {
        fileStream >> tmp;
        ++count;
        c = fileStream.peek();
    }

    fileStream.close();
    return count;
}

double **getMatrix(const char *path, const int n, const int m)
{
    fstream myfile;
    myfile.open(path);

    double **mat = new double *[n];
    for (int i = 0; i < n; i++)
    {
        double *tmp = new double[m];
        mat[i] = tmp;
    }

    string tmpStr;
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        {
            myfile >> tmpStr; //
            double dValue = atof(tmpStr.c_str());
            mat[i][j] = dValue;
        }
    }
    return mat;
}

std::array<double, 3> extractCoordinates(const std::string &input)
{
    size_t start = input.find('[');
    size_t end = input.find(']', start);

    if (start == std::string::npos || end == std::string::npos)
    {
        throw std::invalid_argument("无效输入：未找到 '[' 或 ']'");
    }

    std::string coords_str = input.substr(start + 1, end - start - 1);
    std::stringstream ss(coords_str);
    std::array<double, 3> coords;
    char comma;

    if (!(ss >> coords[0] >> comma >> coords[1] >> comma >> coords[2]))
    {
        throw std::invalid_argument("无效输入：坐标格式错误");
    }



    return coords;
}

Eigen::Matrix3d Yaw_Rotation(float yaw)
{
    Eigen::Matrix3d T;
    T << cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1;
    return T;
}

Eigen::Matrix3d Pitch_Rotation(float pitch)
{
    Eigen::Matrix3d T;
    T << cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch);
    return T;
}

Eigen::Matrix3d Roll_Rotation(float roll)
{
    Eigen::Matrix3d T;
    T << 1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll);
    return T;
}

Eigen::Matrix4d neck_to_eye(int flag)
{
    int id30_position, id31_position, id32_position;
    // getPosition(hang_waist_id, 30, id30_position);
    // getPosition(hang_waist_id, 31, id31_position);
    // getPosition(hang_waist_id, 32, id32_position);

    id30_position = 0;
    // id31_position = 0;

    if (flag == 0)
    {
        id31_position = -20058;
    }

    else if (flag == 3)
    {
        id31_position = 10000;
    }
    else if (flag == 4)
    {
        id31_position = -10000;
    }
    else
    {
        id31_position = -22058;
    }

    id32_position = -0;

    // cout << "id30_position: " << id30_position << " " << "id31_position: " << id31_position << " " << "id32_position: " << id32_position << endl;
    float r_30 = id30_position / ((4 * 65536) / (2 * M_PI));
    float r_31 = id31_position / ((4 * 65536) / (2 * M_PI));
    float r_32 = id32_position / ((4 * 65536) / (2 * M_PI));
    // cout << "r_30: " << r_30 << " " << "r_31: " << r_31 << " " << "r_32: " << r_32 << endl;

    Eigen::Matrix4d T1 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T2 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T3 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T4 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T5 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T6 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T7 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T8 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T9 = Eigen::Matrix4d::Identity();
    // Eigen::Vector3d
    // T1.block<3, 1>(0, 3) = pos;
    T1.block<3, 3>(0, 0) = Yaw_Rotation(r_30);
    // Eigen::Vector3d diff1; diff1 << 0, 0, 0;
    Eigen::Vector3d diff1;
    diff1 << 0, 0, 145;
    T2.block<3, 1>(0, 3) = diff1;
    T3.block<3, 3>(0, 0) = Roll_Rotation(M_PI / 2);
    T4.block<3, 3>(0, 0) = Yaw_Rotation(r_31);
    T5.block<3, 3>(0, 0) = Pitch_Rotation(M_PI / 2);
    T6.block<3, 3>(0, 0) = Yaw_Rotation(r_32);
    Eigen::Vector3d diff2;
    diff2 << 30, 135, 32;
    // Eigen::Vector3d diff2; diff2 << 35, 12, 95;
    T7.block<3, 1>(0, 3) = diff2;
    T8.block<3, 3>(0, 0) = Roll_Rotation(15 * M_PI / 180);
    T9.block<3, 3>(0, 0) = Yaw_Rotation(M_PI);

    return T1 * T2 * T3 * T4 * T5 * T6 * T7 * T8 * T9;
}

double calculateTf(const Eigen::Matrix<double, 1, 6> &axis_p1,
                   const Eigen::Matrix<double, 1, 6> &axis_p2,
                   double V_DEFAULT)
{
    // // 1. 提取平移和旋转分量
    // Eigen::Vector3d trans_p1 = axis_p1.head(3);  // 起始平移 (x,y,z)
    // Eigen::Vector3d trans_p2 = axis_p2.head(3);  // 目标平移 (x,y,z)
    // Eigen::Vector3d rot_p1 = axis_p1.tail(3);    // 起始旋转 (rx,ry,rz)
    // Eigen::Vector3d rot_p2 = axis_p2.tail(3);    // 目标旋转 (rx,ry,rz)

    Matrix<double, 1, 6> axis_p;
    Matrix<double, 4, 4> T_p1_p2, T_p1, T_p2;
    T_p1 = Axispos2T(axis_p1);
    T_p2 = Axispos2T(axis_p2);
    axis_p = T2Axispos(T_p1.inverse() * T_p2);

    // 2. 计算平移距离和旋转等效距离
    double d_trans = (axis_p.head(3)).norm(); // 平移距离 (m)
    double d_rot = (axis_p.tail(3)).norm();   // 旋转角度差 (rad)
    double d_rot_eq = 0.2 * d_rot;            // 旋转等效距离 (特征长度0.5m)

    // 3. 处理微小距离（无需运动）
    const double EPS = 1e-6; // 微小距离阈值 (m)
    bool trans_need_move = (d_trans > EPS);
    bool rot_need_move = (d_rot_eq > EPS);

    if (!trans_need_move && !rot_need_move)
    {
        return 0.0; // 已到达目标，无需运动
    }

    // 4. 分别计算平移和旋转的理论时间
    double t_trans = trans_need_move ? (d_trans / V_DEFAULT) : 0.0;
    double t_rot = rot_need_move ? (d_rot_eq / V_DEFAULT) : 0.0;

    // 5. 总时间取最大值（确保同步完成）
    double Tf = std::max(t_trans, t_rot);

    // 6. 强制最小时间约束（避免超短时间导致冲击）
    const double MIN_TF = 0.1; // 最小运动时间 (s)，可根据机械臂特性调整
    if (Tf < MIN_TF)
    {
        Tf = MIN_TF;
    }

    // 调试信息（可选）
    // std::cout << "平移距离: " << d_trans*1000 << "mm, 旋转等效距离: " << d_rot_eq*1000 << "mm\n";
    // std::cout << "平移时间: " << t_trans << "s, 旋转时间: " << t_rot << "s, 总时间: " << Tf << "s\n";

    return Tf;
}

Eigen::Matrix<double, 1, 7> joint_range_normalize(
    const Eigen::Matrix<double, 1, 7> &q,
    const Eigen::Matrix<double, 1, 7> &q_min,
    const Eigen::Matrix<double, 1, 7> &q_max)
{
    // 计算中点和单侧范围（向量化操作）
    Matrix<double, 1, 7> q_mid = (q_min + q_max) / 2.0;
    Matrix<double, 1, 7> half_range = (q_max - q_min) / 2.0;

    // 避免除零（向量化处理）
    half_range = half_range.cwiseMax(1e-6);

    // 计算归一化距离并限制在[0,1]（单步完成核心逻辑）
    return (q - q_mid).cwiseAbs().cwiseQuotient(half_range).cwiseMax(0.0).cwiseMin(1.0);
}

Eigen::Matrix<double, 1, 7> calc_segment_weight(
    const Eigen::Matrix<double, 1, 7> &x,
    double w_max)
{
    // 参数校验：w_max必须为正标量
    if (w_max <= 0)
    {
        throw std::invalid_argument("w_max必须是正标量（如100）！");
    }

    // 核心参数（常量表达式，编译时确定）
    constexpr double safe_threshold = 0.7;          // 安全区与警戒区分界点
    constexpr double safe_w_base = 1e-3;            // 安全区基础权重
    constexpr double growth_factor = 5;             // 警戒区增长系数
    constexpr double interval = 1 - safe_threshold; // 警戒区区间长度（0.3）

    // 初始化输出矩阵（1×7固定尺寸，无需手动指定大小）
    Eigen::Matrix<double, 1, 7> w;

    // 循环处理每个维度（利用固定尺寸特性，循环次数编译时确定）
    for (int i = 0; i < 7; ++i)
    {
        const double xi = x(i); // 访问1×7矩阵的第i列元素（0~6）

        if (xi <= safe_threshold)
        {
            // 安全区：x ≤ 0.7 → 基础权重
            w(i) = safe_w_base;
        }
        else if (xi < 1.0)
        {
            // 警戒区：0.7 < x < 1 → 指数增长到w_max
            const double ratio = (xi - safe_threshold) / interval; // 0~1比例
            w(i) = safe_w_base + (w_max - safe_w_base) * (1 - std::exp(-growth_factor * ratio));
        }
        else
        {
            // 超限区：x ≥ 1 → 最大权重
            w(i) = w_max;
        }
    }

    return w;
}


std::tuple<MatrixXd, MatrixXd, MatrixXd>
quinticInterp(
    double t,
    double totalTime,
    const MatrixXd& start,  // 统一 Matrixd：1×N
    const MatrixXd& target  // 统一 Matrixd：1×N
) {
    // 1. 输入合法性校验（新增“必须是1行”校验，适配关节数据）
    // 总时间为0：返回终点+零速度/加速度
    if (totalTime <= 1e-6) {
        MatrixXd zero_mat(1, start.cols());
        zero_mat.setZero();
        return {target, zero_mat, zero_mat};
    }
    // 起点/终点必须是1行（避免多行矩阵输入）
    if (start.rows() != 1 || target.rows() != 1) {
        MatrixXd zero_mat(1, start.cols());
        zero_mat.setZero();
        return {start, zero_mat, zero_mat};
    }
    // 起点/终点列数（关节数）必须一致
    if (start.cols() != target.cols()) {
        MatrixXd zero_mat(1, start.cols());
        zero_mat.setZero();
        return {start, zero_mat, zero_mat};
    }

    // 2. 时间边界夹紧（避免超界）
    double clampedTime = std::max(0.0, std::min(t, totalTime));
    double ratio = clampedTime / totalTime;  // 时间占比（0~1）

    // 3. 五次多项式核心项预计算（无冲击插值，工业级平滑）
    const double r2 = ratio * ratio;
    const double r3 = r2 * ratio;
    const double r4 = r3 * ratio;
    const double r5 = r4 * ratio;

    // 位置/速度/加速度项（解析导数，保证连续）
    const double pos_term = 10 * r3 - 15 * r4 + 6 * r5;
    const double vel_term = (30 * r2 - 60 * r3 + 30 * r4) / totalTime;
    const double acc_term = (60 * ratio - 180 * r2 + 120 * r3) / (totalTime * totalTime);

    // 4. 计算结果（Matrixd 直接运算，维度自动匹配 1×N）
    MatrixXd delta = target - start;  // 位置差（1×N）
    MatrixXd pos = start + delta * pos_term;
    MatrixXd vel = delta * vel_term;
    MatrixXd acc = delta * acc_term;

    return {pos, vel, acc};
}

int g_arm_last_cmd_cnt[2][7] = {};
bool g_arm_last_cmd_valid[2] = {false, false};

void save_arm_last_commanded_position(int hand, const int motor_cmd[7])
{
    if (hand < 0 || hand > 1)
        return;
    for (int i = 0; i < 7; ++i)
        g_arm_last_cmd_cnt[hand][i] = motor_cmd[i];
    g_arm_last_cmd_valid[hand] = true;
}

namespace
{

int arm_hand_side(const Robot_Arm &arm)
{
    if (!arm.can_id_list.empty() && arm.can_id_list[0] >= 23)
        return 0;
    return 1;
}

double encoder_cnt_to_rad(int cnt)
{
    const double deg = static_cast<double>(cnt) * 360.0 / 65536.0 / 4.0;
    return deg * M_PI / 180.0;
}

int rad_to_encoder_cnt(double q_rad)
{
    const double deg = q_rad * 180.0 / M_PI;
    return static_cast<int>(deg * 65536.0 * 4.0 / 360.0);
}

constexpr double k_arm_traj_dt = 5e-3;
constexpr int kMotorPositionReachTolerance = 550;

bool arm_motors_at_commanded_position(int hand, int tolerance = kMotorPositionReachTolerance)
{
    if (hand < 0 || hand > 1 || !g_arm_last_cmd_valid[hand])
        return true;

    int actual[7] = {0};
    get_motor_position(actual, hand);
    for (int i = 0; i < 7; ++i)
    {
        if (std::abs(actual[i] - g_arm_last_cmd_cnt[hand][i]) > tolerance)
            return false;
    }
    return true;
}

void log_arms_motor_position_reach(bool log_r, bool log_l, const char *reason)
{
    std::cout << "[arm] " << reason;
    auto log_one = [](int hand, const char *tag) {
        if (!g_arm_last_cmd_valid[hand])
        {
            std::cout << ' ' << tag << "=no_cmd";
            return;
        }
        int actual[7] = {0};
        get_motor_position(actual, hand);
        std::cout << ' ' << tag << "_err=";
        for (int i = 0; i < 7; ++i)
        {
            std::cout << (actual[i] - g_arm_last_cmd_cnt[hand][i]);
            if (i + 1 < 7)
                std::cout << ',';
        }
    };
    if (log_r)
        log_one(1, "pos_r");
    if (log_l)
        log_one(0, "pos_l");
    std::cout << std::endl << std::flush;
}

Eigen::MatrixXd make_linear_joint_traj(
    const Eigen::MatrixXd &q_start,
    const Eigen::MatrixXd &q_end,
    double dt)
{
    const int dof = static_cast<int>(std::min({q_start.cols(), q_end.cols(), static_cast<Eigen::Index>(7)}));
    double max_dq = 0.0;
    for (int j = 0; j < dof; ++j)
        max_dq = std::max(max_dq, std::abs(q_end(0, j) - q_start(0, j)));
    constexpr double kDegPerSec = 30.0;
    const double t_s = std::max(0.4, (max_dq * 180.0 / M_PI) / kDegPerSec);
    const int n = std::max(2, static_cast<int>(std::ceil(t_s / dt)) + 1);
    Eigen::MatrixXd traj(n, dof);
    for (int i = 0; i < n; ++i)
    {
        const double s = static_cast<double>(i) / static_cast<double>(n - 1);
        const double e = 0.5 * (1.0 - std::cos(M_PI * s));
        for (int j = 0; j < dof; ++j)
            traj(i, j) = q_start(0, j) + e * (q_end(0, j) - q_start(0, j));
    }
    return traj;
}

int execute_arm_trajectory(const MatrixXd &traj, int hand, double dt = k_arm_traj_dt)
{
    if (hand == 1 && right_arm_motors_locked())
        return 0;
    if (traj.rows() == 0)
    {
        cout << "[Error] Trajectory is empty, execution aborted." << endl;
        return -1;
    }

    const int dof = static_cast<int>(std::min(traj.cols(), static_cast<Eigen::Index>(7)));
    for (int i = 0; i < traj.rows(); ++i)
    {
        if (hardware_abort_requested())
        {
            std::cout << "[arm] STOP，停止轨迹\n";
            return -4;
        }
        if (can_io_faulted())
        {
            std::cout << "[arm] CAN 收发失败，停止轨迹: " << can_io_fault_message() << std::endl;
            return -2;
        }
        const auto cycle_start = chrono::high_resolution_clock::now();

        int motor_cmd[7] = {0};
        double q_rad[7] = {0, 0, 0, 0, 0, 0, 0};
        for (int j = 0; j < dof; ++j)
        {
            q_rad[j] = traj(i, j);
            motor_cmd[j] = rad_to_encoder_cnt(traj(i, j));
        }
        hand_shadow_note_q(hand, q_rad);
        set_motor_position(motor_cmd, hand);

        const double elapsed_ms =
            chrono::duration<double, milli>(chrono::high_resolution_clock::now() - cycle_start).count();
        const double sleep_ms = std::max(0.0, dt * 1000.0 - elapsed_ms);
        if (sleep_ms > 1e-6)
            this_thread::sleep_for(chrono::duration<double, milli>(sleep_ms));
    }

    int last_cmd[7] = {0};
    for (int j = 0; j < dof; ++j)
        last_cmd[j] = rad_to_encoder_cnt(traj(traj.rows() - 1, j));
    save_arm_last_commanded_position(hand, last_cmd);
    return 0;
}

Eigen::MatrixXd arm_read_motor_joints(int hand)
{
    int motor_cnt[7] = {0};
    get_motor_position(motor_cnt, hand);

    Eigen::MatrixXd q_start(1, 7);
    for (int i = 0; i < 7; ++i)
        q_start(0, i) = encoder_cnt_to_rad(motor_cnt[i]);
    return q_start;
}

Matrix<double, 1, 6> arm_flange_tool(const Robot_Arm &arm)
{
    Matrix<double, 1, 6> tool;
    if (arm_hand_side(arm) == 1)
        tool << 0.26, 0, 0, 0, 0, 0;
    else
        tool << 0.26, 0, 0, 0, 0, 0;
    return tool;
}

using WaveQ = Eigen::Matrix<double, 1, 7>;

WaveQ wave_q(double j1, double j2, double j3, double j4, double j5, double j6, double j7)
{
    WaveQ q;
    q << j1, j2, j3, j4, j5, j6, j7;
    return q;
}

const WaveQ k_wave_hang = wave_q(-0.0140, 1.3142, -0.5882, 0.2129, -0.5777, 0.3019, 0.0925);
const WaveQ k_wave_front = wave_q(0.0541, 1.0385, -0.6562, 1.0437, -0.3019, 0.6109, -0.2601);
const WaveQ k_wave_raise = wave_q(0.2286, 1.0071, 0.2548, 1.7924, 1.1519, 0.6109, 0.9215);
const WaveQ k_wave_out = wave_q(0.2286, 1.0071, 0.3752, 1.6825, 1.3980, 0.6109, 0.8866);
const WaveQ k_wave_in = wave_q(0.2304, 1.0071, 0.1344, 1.9007, 0.9058, 0.6109, 0.9564);
constexpr double k_wave_amp = 1.4;
constexpr double k_wave_head_pitch_deg = 10.0;
constexpr double k_wave_grip_ready = 0.3;
constexpr double k_wave_period_s = 1.1;
constexpr double k_wave_cycles = 3.0;
constexpr double k_wave_skip_rad = 2.0 * M_PI / 180.0;

const double k_wave_right_min_deg[7] = {-90, -90, -90, -10, -180, -40, -20};
const double k_wave_right_max_deg[7] = {180, 5, 90, 140, 180, 40, 90};
const double k_wave_left_min_deg[7] = {-90, 5, -90, -10, -90, -40, -90};
const double k_wave_left_max_deg[7] = {180, 90, 90, 140, 90, 40, 20};

double wave_model_rad(int hand, int joint, double q_motor)
{
    double dir = 1.0;
    double off = 0.0;
    if (hand == 1)
    {
        if (joint == 1)
            off = M_PI / 2.0;
    }
    else
    {
        if (joint == 0 || joint == 3 || joint == 5)
            dir = -1.0;
        if (joint == 1)
            off = -M_PI / 2.0;
    }
    return (q_motor - off) / dir;
}

int wave_log_over_limit(Robot_Arm &arm, int hand, const WaveQ &q, const char *tag)
{
    if (!arm.Check_Joint_Limit(q))
        return 0;
    const double *mn = hand == 1 ? k_wave_right_min_deg : k_wave_left_min_deg;
    const double *mx = hand == 1 ? k_wave_right_max_deg : k_wave_left_max_deg;
    bool named = false;
    for (int j = 0; j < 7; ++j)
    {
        const double model_deg = wave_model_rad(hand, j, q(j)) * 180.0 / M_PI;
        if (model_deg >= mn[j] - 1e-3 && model_deg <= mx[j] + 1e-3)
            continue;
        named = true;
        std::cerr << std::fixed << std::setprecision(2)
                  << "[wave] " << (hand == 1 ? "右臂" : "左臂")
                  << (tag ? tag : "") << " J" << (j + 1)
                  << " 超限 电机角 " << (q(j) * 180.0 / M_PI)
                  << "° 模型角(conv0) " << model_deg
                  << "° 限位 [" << mn[j] << ", " << mx[j] << "]°\n";
    }
    if (!named)
    {
        std::cerr << "[wave] " << (hand == 1 ? "右臂" : "左臂")
                  << (tag ? tag : "") << " Check_Joint_Limit 超限\n";
    }
    return -1;
}

int wave_check_traj(Robot_Arm &arm, int hand, const Eigen::MatrixXd &traj, const char *tag,
                    bool skip_start = false)
{
    const int i0 = (skip_start && traj.rows() > 1) ? 1 : 0;
    for (int i = i0; i < traj.rows(); ++i)
    {
        WaveQ q;
        for (int j = 0; j < 7; ++j)
            q(j) = traj(i, j);
        if (wave_log_over_limit(arm, hand, q, tag) != 0)
            return -1;
    }
    return 0;
}

Eigen::MatrixXd wave_quintic(const WaveQ &qa, const WaveQ &qb, double duration_s)
{
    const int n = std::max(2, static_cast<int>(std::lround(duration_s / k_arm_traj_dt)) + 1);
    Eigen::MatrixXd traj(n, 7);
    for (int i = 0; i < n; ++i)
    {
        const double u = static_cast<double>(i) / static_cast<double>(n - 1);
        const double u2 = u * u;
        const double u3 = u2 * u;
        const double s = 10.0 * u3 - 15.0 * u2 * u2 + 6.0 * u2 * u3;
        for (int j = 0; j < 7; ++j)
            traj(i, j) = qa(j) + s * (qb(j) - qa(j));
    }
    return traj;
}

Eigen::MatrixXd wave_sine_traj()
{
    const double total = k_wave_period_s * k_wave_cycles;
    const double half = k_wave_period_s * 0.5;
    const WaveQ amp = k_wave_amp * (k_wave_out - k_wave_raise);
    const int n = std::max(2, static_cast<int>(std::lround(total / k_arm_traj_dt)) + 1);
    Eigen::MatrixXd traj(n, 7);
    for (int i = 0; i < n; ++i)
    {
        const double t = std::min(total, static_cast<double>(i) * k_arm_traj_dt);
        const double phase = 2.0 * M_PI * t / k_wave_period_s;
        double env = 1.0;
        if (t < half)
            env = 0.5 * (1.0 - std::cos(M_PI * t / half));
        else if (t > total - half)
            env = 0.5 * (1.0 - std::cos(M_PI * (total - t) / half));
        const double s = env * std::sin(phase);
        for (int j = 0; j < 7; ++j)
            traj(i, j) = k_wave_raise(j) + s * amp(j);
    }
    return traj;
}

int wave_pause_ms(int ms)
{
    hardware_abort_sleep_ms(ms);
    if (hardware_abort_requested())
        return -4;
    return 0;
}

Eigen::MatrixXd wave_via(const WaveQ &q0, const WaveQ &q1, const WaveQ &q2, double t1, double t2)
{
    auto eval = [](double t, double T, double q_a, double q_b, double v_a, double v_b) {
        const double T2 = T * T;
        const double T3 = T2 * T;
        const double T4 = T3 * T;
        const double T5 = T4 * T;
        const double a0 = q_a;
        const double a1 = v_a;
        const double a2 = 0.0;
        const double a3 = (20.0 * (q_b - q_a) - (8.0 * v_b + 12.0 * v_a) * T) / (2.0 * T3);
        const double a4 = (30.0 * (q_a - q_b) + (14.0 * v_b + 16.0 * v_a) * T) / (2.0 * T4);
        const double a5 = (12.0 * (q_b - q_a) - 6.0 * (v_b + v_a) * T) / (2.0 * T5);
        const double t2p = t * t;
        const double t3p = t2p * t;
        return a0 + a1 * t + a2 * t2p + a3 * t3p + a4 * t2p * t2p + a5 * t2p * t3p;
    };
    const int n1 = std::max(2, static_cast<int>(std::lround(t1 / k_arm_traj_dt)) + 1);
    const int n2 = std::max(2, static_cast<int>(std::lround(t2 / k_arm_traj_dt)) + 1);
    Eigen::MatrixXd traj(n1 + n2 - 1, 7);
    WaveQ v_mid;
    for (int j = 0; j < 7; ++j)
    {
        const double v_in = (q1(j) - q0(j)) / t1;
        const double v_out = (q2(j) - q1(j)) / t2;
        v_mid(j) = 0.5 * (v_in + v_out);
    }
    for (int i = 0; i < n1; ++i)
    {
        const double t = t1 * static_cast<double>(i) / static_cast<double>(n1 - 1);
        for (int j = 0; j < 7; ++j)
            traj(i, j) = eval(t, t1, q0(j), q1(j), 0.0, v_mid(j));
    }
    for (int i = 1; i < n2; ++i)
    {
        const double t = t2 * static_cast<double>(i) / static_cast<double>(n2 - 1);
        for (int j = 0; j < 7; ++j)
            traj(n1 + i - 1, j) = eval(t, t2, q1(j), q2(j), v_mid(j), 0.0);
    }
    return traj;
}

int wave_head_zero()
{
    if (head_id < 0)
    {
        std::cerr << "[wave] 头电机未绑定\n";
        return -2;
    }
    for (uint32_t id : kHeadCanIds)
    {
        if (!motor_enable_id(id))
            std::cerr << "[wave] 头使能失败 id=" << id << "，仍尝试回零\n";
    }
    uint32_t ids[3] = {kHeadYawCanId, kHeadPitchCanId, kHeadRollCanId};
    const double pose[3] = {0.0, k_wave_head_pitch_deg, 0.0};
    std::cout << "[wave] 头偏航和横滚回零，俯仰低到 " << k_wave_head_pitch_deg << "°\n";
    if (smooth_motor_move_deg(head_id, 3, ids, pose, 25.0, 20) != 0)
        return -2;
    if (hardware_abort_requested())
        return -4;
    return 0;
}

void wave_waist_yaw_deg(double yaw_deg)
{
    if (waist_id < 0)
        return;
    uint32_t id = 1;
    int cnt = motor_angle_deg_to_cnt(yaw_deg);
    socketcan_sendcommand(waist_id, 1, &id, 30, &cnt);
}

int wave_waist_to(double yaw_deg)
{
    if (waist_id < 0)
    {
        std::cerr << "[welcome] 腰回转未绑定\n";
        return -2;
    }
    uint32_t id = 1;
    const double ang = yaw_deg;
    if (smooth_motor_move_deg(waist_id, 1, &id, &ang, 30.0, 20) != 0)
        return -2;
    if (hardware_abort_requested())
        return -4;
    return 0;
}

double wave_quintic_s(double u)
{
    const double u2 = u * u;
    const double u3 = u2 * u;
    return 10.0 * u3 - 15.0 * u2 * u2 + 6.0 * u2 * u3;
}

int wave_play_lift(int hand, const Eigen::MatrixXd &traj, const std::vector<double> &yaw_deg)
{
    if (hardware_abort_requested())
        return -4;
    if (can_io_faulted())
        return -2;
    if (traj.rows() < 2)
        return 0;
    if (hand == 1 && right_arm_motors_locked())
        return 0;
    const int dof = static_cast<int>(std::min(traj.cols(), static_cast<Eigen::Index>(7)));
    const int n = static_cast<int>(traj.rows());
    for (int i = 0; i < n; ++i)
    {
        if (hardware_abort_requested())
        {
            std::cout << "[arm] STOP，停止轨迹\n";
            return -4;
        }
        if (can_io_faulted())
        {
            std::cout << "[arm] CAN 收发失败，停止轨迹: " << can_io_fault_message() << std::endl;
            return -2;
        }
        const auto cycle_start = std::chrono::high_resolution_clock::now();
        int motor_cmd[7] = {0};
        for (int j = 0; j < dof; ++j)
            motor_cmd[j] = rad_to_encoder_cnt(traj(i, j));
        set_motor_position(motor_cmd, hand);
        if (i < static_cast<int>(yaw_deg.size()))
            wave_waist_yaw_deg(yaw_deg[static_cast<size_t>(i)]);
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::high_resolution_clock::now() - cycle_start)
                .count();
        const double sleep_ms = std::max(0.0, k_arm_traj_dt * 1000.0 - elapsed_ms);
        if (sleep_ms > 1e-6)
            std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(sleep_ms));
    }
    int last_cmd[7] = {0};
    for (int j = 0; j < dof; ++j)
        last_cmd[j] = rad_to_encoder_cnt(traj(traj.rows() - 1, j));
    save_arm_last_commanded_position(hand, last_cmd);
    return 0;
}

void wave_head_pitch_deg(double pitch_deg)
{
    if (head_id < 0)
        return;
    uint32_t ids[3] = {kHeadYawCanId, kHeadPitchCanId, kHeadRollCanId};
    int cnt[3] = {
        motor_angle_deg_to_cnt(0.0),
        motor_angle_deg_to_cnt(pitch_deg),
        motor_angle_deg_to_cnt(0.0),
    };
    socketcan_sendcommand(head_id, 3, ids, 30, cnt);
}

int wave_play(int hand, const Eigen::MatrixXd &traj, bool nod,
              const std::vector<double> *yaw_deg = nullptr)
{
    if (hardware_abort_requested())
        return -4;
    if (can_io_faulted())
        return -2;
    if (traj.rows() < 2)
        return 0;
    if (!nod && yaw_deg == nullptr)
        return execute_arm_trajectory(traj, hand, k_arm_traj_dt);

    if (hand == 1 && right_arm_motors_locked())
        return 0;
    const int dof = static_cast<int>(std::min(traj.cols(), static_cast<Eigen::Index>(7)));
    const double total = k_wave_period_s * k_wave_cycles;
    const double half = k_wave_period_s * 0.5;
    constexpr double k_nod_deg = 8.0;
    for (int i = 0; i < traj.rows(); ++i)
    {
        if (hardware_abort_requested())
        {
            std::cout << "[arm] STOP，停止轨迹\n";
            return -4;
        }
        if (can_io_faulted())
        {
            std::cout << "[arm] CAN 收发失败，停止轨迹: " << can_io_fault_message() << std::endl;
            return -2;
        }
        const auto cycle_start = std::chrono::high_resolution_clock::now();
        int motor_cmd[7] = {0};
        for (int j = 0; j < dof; ++j)
            motor_cmd[j] = rad_to_encoder_cnt(traj(i, j));
        set_motor_position(motor_cmd, hand);
        if (yaw_deg != nullptr && i < static_cast<int>(yaw_deg->size()))
            wave_waist_yaw_deg((*yaw_deg)[static_cast<size_t>(i)]);

        if (nod)
        {
            const double t = std::min(total, static_cast<double>(i) * k_arm_traj_dt);
            const double phase = 2.0 * M_PI * t / k_wave_period_s;
            double env = 1.0;
            if (t < half)
                env = 0.5 * (1.0 - std::cos(M_PI * t / half));
            else if (t > total - half)
                env = 0.5 * (1.0 - std::cos(M_PI * (total - t) / half));
            wave_head_pitch_deg(k_wave_head_pitch_deg +
                                k_nod_deg * env * 0.5 * (1.0 - std::cos(phase)));
        }

        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::high_resolution_clock::now() - cycle_start)
                .count();
        const double sleep_ms = std::max(0.0, k_arm_traj_dt * 1000.0 - elapsed_ms);
        if (sleep_ms > 1e-6)
            std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(sleep_ms));
    }
    int last_cmd[7] = {0};
    for (int j = 0; j < dof; ++j)
        last_cmd[j] = rad_to_encoder_cnt(traj(traj.rows() - 1, j));
    save_arm_last_commanded_position(hand, last_cmd);
    if (nod)
        wave_head_pitch_deg(k_wave_head_pitch_deg);
    return 0;
}

int wave_goto(Robot_Arm &arm, const WaveQ &goal, double fixed_s)
{
    const int hand = arm_hand_side(arm);
    if (hardware_abort_requested())
        return -4;
    if (can_io_faulted())
        return -2;
    const Eigen::MatrixXd q_now = arm_read_motor_joints(hand);
    if (can_io_faulted() || !q_now.allFinite())
        return -2;
    WaveQ qa;
    for (int j = 0; j < 7; ++j)
        qa(j) = q_now(0, j);
    const double max_dq = (qa - goal).cwiseAbs().maxCoeff();
    if (max_dq <= k_wave_skip_rad)
        return 0;
    const double duration = fixed_s > 0.0 ? fixed_s : std::max(1.0, max_dq / 0.45);
    const Eigen::MatrixXd traj = wave_quintic(qa, goal, duration);
    if (wave_check_traj(arm, hand, traj, "", true) != 0)
        return -1;
    return wave_play(hand, traj, false);
}

Eigen::MatrixXd wave_sine_for(const WaveQ &raise, const WaveQ &outward)
{
    const double total = k_wave_period_s * k_wave_cycles;
    const double half = k_wave_period_s * 0.5;
    const WaveQ amp = k_wave_amp * (outward - raise);
    const int n = std::max(2, static_cast<int>(std::lround(total / k_arm_traj_dt)) + 1);
    Eigen::MatrixXd traj(n, 7);
    for (int i = 0; i < n; ++i)
    {
        const double t = std::min(total, static_cast<double>(i) * k_arm_traj_dt);
        const double phase = 2.0 * M_PI * t / k_wave_period_s;
        double env = 1.0;
        if (t < half)
            env = 0.5 * (1.0 - std::cos(M_PI * t / half));
        else if (t > total - half)
            env = 0.5 * (1.0 - std::cos(M_PI * (total - t) / half));
        const double s = env * std::sin(phase);
        for (int j = 0; j < 7; ++j)
            traj(i, j) = raise(j) + s * amp(j);
    }
    return traj;
}

int wave_one(Robot_Arm &arm, bool is_right)
{
    const WaveQ hang = is_right ? k_wave_hang : WaveQ(-k_wave_hang);
    const WaveQ front = is_right ? k_wave_front : WaveQ(-k_wave_front);
    const WaveQ raise = is_right ? k_wave_raise : WaveQ(-k_wave_raise);
    const WaveQ outward = is_right ? k_wave_out : WaveQ(-k_wave_out);
    const int hand = is_right ? 1 : 0;
    const char *name = is_right ? "右手" : "左手";

    WaveQ q_from = hang;
    const Eigen::MatrixXd q_now = arm_read_motor_joints(hand);
    if (can_io_faulted() || !q_now.allFinite())
        return -2;
    for (int j = 0; j < 7; ++j)
        q_from(j) = q_now(0, j);

    constexpr double k_lift_front_s = 1.0;
    constexpr double k_lift_raise_s = 0.8;
    constexpr double k_pause_s = 0.25;
    constexpr double k_lower_front_s = 0.9;
    constexpr double k_lower_hang_s = 1.1;
    constexpr double k_welcome_waist_deg = 20.0;
    const double wave_s = k_wave_period_s * k_wave_cycles;
    const double yaw_here = is_right ? -k_welcome_waist_deg : k_welcome_waist_deg;
    const double yaw_across = -yaw_here;
    const double t_side = wave_s;
    const double t_across = t_side + wave_s;
    const double t_end = t_across + wave_s;
    auto yaw_at = [&](double t) {
        if (t <= 0.0)
            return 0.0;
        if (t < t_side)
            return wave_quintic_s(t / t_side) * yaw_here;
        if (t < t_across)
        {
            const double u = (t - t_side) / (t_across - t_side);
            return yaw_here + wave_quintic_s(u) * (yaw_across - yaw_here);
        }
        if (t < t_end)
        {
            const double u = (t - t_across) / (t_end - t_across);
            return yaw_across + wave_quintic_s(u) * (0.0 - yaw_across);
        }
        return 0.0;
    };
    auto yaw_for = [&](double t0, int n) {
        std::vector<double> y(static_cast<size_t>(std::max(0, n)), 0.0);
        for (int i = 0; i < n; ++i)
            y[static_cast<size_t>(i)] = yaw_at(t0 + static_cast<double>(i) * k_arm_traj_dt);
        return y;
    };
    std::cout << "[welcome] " << name
              << " 腰 ±" << k_welcome_waist_deg
              << "°，开始转到挥手侧、再转到另一侧、最后回 0，每段 "
              << wave_s << "s，和挥三下一样长\n";
    enable_waist_motors();
    const Eigen::MatrixXd lift = wave_via(q_from, front, raise, k_lift_front_s, k_lift_raise_s);
    if (wave_check_traj(arm, hand, lift, " 抬手") != 0)
        return -1;
    double t_cursor = 0.0;
    const std::vector<double> yaw_lift = yaw_for(t_cursor, static_cast<int>(lift.rows()));
    int rc = wave_play_lift(hand, lift, yaw_lift);
    t_cursor += static_cast<double>(lift.rows()) * k_arm_traj_dt;
    if (rc != 0)
        return rc;
    const int pause_n = std::max(1, static_cast<int>(std::lround(k_pause_s / k_arm_traj_dt)));
    for (int i = 0; i < pause_n; ++i)
    {
        if (hardware_abort_requested())
            return -4;
        wave_waist_yaw_deg(yaw_at(t_cursor));
        t_cursor += k_arm_traj_dt;
        std::this_thread::sleep_for(std::chrono::duration<double>(k_arm_traj_dt));
    }
    orch::wave_grip_ratio(is_right, k_wave_grip_ready);
    if (hardware_abort_requested())
        return -4;
    std::cout << "[welcome] " << name << " 挥 3 下\n";
    const Eigen::MatrixXd sine = wave_sine_for(raise, outward);
    if (wave_check_traj(arm, hand, sine, " 挥动") != 0)
    {
        orch::wave_grip_ratio(is_right, 1.0);
        return -1;
    }
    const std::vector<double> yaw_sine = yaw_for(t_cursor, static_cast<int>(sine.rows()));
    rc = wave_play(hand, sine, true, &yaw_sine);
    t_cursor += static_cast<double>(sine.rows()) * k_arm_traj_dt;
    if (rc != 0)
    {
        orch::wave_grip_ratio(is_right, 1.0);
        return rc;
    }
    std::cout << "[welcome] " << name << " 放下\n";
    const Eigen::MatrixXd lower = wave_via(raise, front, hang, k_lower_front_s, k_lower_hang_s);
    if (wave_check_traj(arm, hand, lower, " 放下") != 0)
    {
        orch::wave_grip_ratio(is_right, 1.0);
        return -1;
    }
    const std::vector<double> yaw_lower = yaw_for(t_cursor, static_cast<int>(lower.rows()));
    rc = wave_play(hand, lower, false, &yaw_lower);
    t_cursor += static_cast<double>(lower.rows()) * k_arm_traj_dt;
    orch::wave_grip_ratio(is_right, 1.0);
    if (rc != 0)
        return rc;
    while (t_cursor < t_end)
    {
        if (hardware_abort_requested())
            return -4;
        wave_waist_yaw_deg(yaw_at(t_cursor));
        t_cursor += k_arm_traj_dt;
        std::this_thread::sleep_for(std::chrono::duration<double>(k_arm_traj_dt));
    }
    wave_waist_yaw_deg(0.0);
    return 0;
}

struct BowWaist
{
    double ankle = 0.0;
    double knee = 0.0;
    double hip = 0.0;
    double roll = 0.0;
    double yaw = 0.0;
};

BowWaist bow_read_waist()
{
    int cnt[5] = {0, 0, 0, 0, 0};
    get_motor_waist_position(cnt);
    auto geo = [](int c, double motor_to_geo) {
        return motor_to_geo * motor_cnt_to_angle_deg(c) * M_PI / 180.0;
    };
    BowWaist q;
    q.ankle = geo(cnt[0], -1.0);
    q.knee = geo(cnt[1], -1.0);
    q.hip = geo(cnt[2], 1.0);
    q.roll = geo(cnt[3], 1.0);
    q.yaw = geo(cnt[4], 1.0);
    return q;
}

void bow_send_waist(const BowWaist &q)
{
    const double motor_deg[5] = {
        -q.ankle * 180.0 / M_PI,
        -q.knee * 180.0 / M_PI,
        q.hip * 180.0 / M_PI,
        q.roll * 180.0 / M_PI,
        q.yaw * 180.0 / M_PI,
    };
    int cnt[5] = {0, 0, 0, 0, 0};
    for (int i = 0; i < 5; ++i)
        cnt[i] = motor_angle_deg_to_cnt(motor_deg[i]);
    set_waist_motor_position(cnt);
}

BowWaist bow_blend_waist(const BowWaist &a, const BowWaist &b, double s)
{
    BowWaist q;
    q.ankle = a.ankle + s * (b.ankle - a.ankle);
    q.knee = a.knee + s * (b.knee - a.knee);
    q.hip = a.hip + s * (b.hip - a.hip);
    q.roll = a.roll + s * (b.roll - a.roll);
    q.yaw = a.yaw + s * (b.yaw - a.yaw);
    return q;
}

WaveQ bow_blend_arm(const WaveQ &a, const WaveQ &b, double s)
{
    return a + s * (b - a);
}

int bow_play(const WaveQ &r0, const WaveQ &r1, const WaveQ &l0, const WaveQ &l1,
             const BowWaist &w0, const BowWaist &w1, double head0_deg, double head1_deg,
             double duration_s)
{
    enable_waist_motors();
    const int n = std::max(2, static_cast<int>(std::lround(duration_s / k_arm_traj_dt)) + 1);
    for (int i = 0; i < n; ++i)
    {
        if (hardware_abort_requested())
            return -4;
        if (can_io_faulted())
            return -2;
        const auto cycle_start = std::chrono::high_resolution_clock::now();
        const double u = static_cast<double>(i) / static_cast<double>(n - 1);
        const double s = wave_quintic_s(u);
        const WaveQ right = bow_blend_arm(r0, r1, s);
        const WaveQ left = bow_blend_arm(l0, l1, s);
        int cmd_r[7] = {0};
        int cmd_l[7] = {0};
        for (int j = 0; j < 7; ++j)
        {
            cmd_r[j] = rad_to_encoder_cnt(right(j));
            cmd_l[j] = rad_to_encoder_cnt(left(j));
        }
        set_motor_position(cmd_r, 1);
        set_motor_position(cmd_l, 0);
        bow_send_waist(bow_blend_waist(w0, w1, s));
        wave_head_pitch_deg(head0_deg + s * (head1_deg - head0_deg));
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::high_resolution_clock::now() - cycle_start)
                .count();
        const double sleep_ms = std::max(0.0, k_arm_traj_dt * 1000.0 - elapsed_ms);
        if (sleep_ms > 1e-6)
            std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(sleep_ms));
    }
    int last_r[7] = {0};
    int last_l[7] = {0};
    for (int j = 0; j < 7; ++j)
    {
        last_r[j] = rad_to_encoder_cnt(r1(j));
        last_l[j] = rad_to_encoder_cnt(l1(j));
    }
    save_arm_last_commanded_position(1, last_r);
    save_arm_last_commanded_position(0, last_l);
    return 0;
}

Eigen::MatrixXd bow_rest_via(const WaveQ &q0, const WaveQ &q1, const WaveQ &q2, double t1, double t2)
{
    const Eigen::MatrixXd a = wave_quintic(q0, q1, t1);
    const Eigen::MatrixXd b = wave_quintic(q1, q2, t2);
    Eigen::MatrixXd traj(a.rows() + b.rows() - 1, 7);
    traj.topRows(a.rows()) = a;
    traj.bottomRows(b.rows() - 1) = b.bottomRows(b.rows() - 1);
    return traj;
}

} // namespace

bool arm_at_last_command(bool is_right)
{
    if (is_right && right_arm_motors_locked())
        return true;
    return arm_motors_at_commanded_position(is_right ? 1 : 0);
}

void arm_note_last_commanded(int hand)
{
    if (hand < 0 || hand > 1 || !g_arm_last_cmd_valid[hand])
        return;
    double q[7] = {};
    for (int j = 0; j < 7; ++j)
        q[j] = encoder_cnt_to_rad(g_arm_last_cmd_cnt[hand][j]);
    hand_shadow_note_q(hand, q);
}

int arm_creep_unreached_joints(bool move_r, bool move_l)
{
    if (right_arm_motors_locked())
        move_r = false;
    if (!move_r && !move_l)
        return 0;

    auto creep_one = [](int hand) -> int {
        if (hand < 0 || hand > 1 || !g_arm_last_cmd_valid[hand])
            return 0;
        if (arm_motors_at_commanded_position(hand))
            return 0;

        int actual[7] = {0};
        int target[7] = {0};
        get_motor_position(actual, hand);
        int max_err = 0;
        for (int j = 0; j < 7; ++j)
        {
            target[j] = g_arm_last_cmd_cnt[hand][j];
            max_err = std::max(max_err, std::abs(actual[j] - target[j]));
        }
        Eigen::MatrixXd q0(1, 7);
        Eigen::MatrixXd q1(1, 7);
        for (int j = 0; j < 7; ++j)
        {
            q0(0, j) = encoder_cnt_to_rad(actual[j]);
            q1(0, j) = encoder_cnt_to_rad(target[j]);
        }
        const double max_rad = (q1 - q0).cwiseAbs().maxCoeff();
        constexpr double k_slow_rad_s = 8.0 * M_PI / 180.0;
        const double duration = std::max(0.8, max_rad / k_slow_rad_s);
        const int n = std::max(2, static_cast<int>(std::lround(duration / k_arm_traj_dt)) + 1);
        Eigen::MatrixXd traj(n, 7);
        for (int i = 0; i < n; ++i)
        {
            const double u = static_cast<double>(i) / static_cast<double>(n - 1);
            const double u2 = u * u;
            const double u3 = u2 * u;
            const double s = 10.0 * u3 - 15.0 * u2 * u2 + 6.0 * u2 * u3;
            traj.row(i) = q0 + s * (q1 - q0);
        }
        std::cout << "[grasp] " << (hand == 1 ? "右" : "左")
                  << " 关节未进 ±550，慢速补到位 max_err=" << max_err
                  << " 用时 " << std::fixed << std::setprecision(2) << duration << "s\n";
        const int rc = execute_arm_trajectory(traj, hand, k_arm_traj_dt);
        save_arm_last_commanded_position(hand, target);
        return rc;
    };

    int rc_r = 0;
    int rc_l = 0;
    if (move_r && move_l)
    {
        std::thread th_r([&]() { rc_r = creep_one(1); });
        std::thread th_l([&]() { rc_l = creep_one(0); });
        th_r.join();
        th_l.join();
    }
    else if (move_r)
        rc_r = creep_one(1);
    else if (move_l)
        rc_l = creep_one(0);
    if (rc_r != 0)
        return rc_r;
    return rc_l;
}

void clamp_cart_goal_z(Matrix<double, 1, 6> &pos, const char *where)
{
    // 与 grasp_valid.z_min 一致。旧硬编码 -0.33/-0.30 会把下压目标抬回去，空中合爪。
    const double z_min = g_move_cfg.grasp_valid.z_min;
    if (pos(2) >= z_min)
        return;

    const char *loc = (where != nullptr && where[0] != '\0') ? where : "unknown";
    std::cerr << "\033[31m[arm][WARN] goal z=" << std::fixed << std::setprecision(4) << pos(2)
              << " 低于 grasp_valid.z_min=" << z_min << "，位置: " << loc << "，钳位为 " << z_min
              << "\033[0m" << std::endl;
    pos(2) = z_min;
}

const char *g_arm_line_move_debug_stage = nullptr;
std::mutex g_arm_line_move_stage_mu;

void arm_line_move_set_debug_stage(const char *stage)
{
    std::lock_guard<std::mutex> lock(g_arm_line_move_stage_mu);
    g_arm_line_move_debug_stage = stage;
}

namespace
{

constexpr const char *kLineTrajFailDebugSep =
    "================================================================================\n";

std::mutex g_line_traj_fail_log_mu;

const char *arm_hand_label(int hand)
{
    return hand == 1 ? "right" : "left";
}

std::string pose6_to_csv(const Matrix<double, 1, 6> &p)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    for (int i = 0; i < 6; ++i)
    {
        if (i > 0)
            oss << ',';
        oss << p(i);
    }
    return oss.str();
}

} // namespace

void append_line_trajectory_fail_debug(
    const char *hand_label,
    int ret,
    const Matrix<double, 1, 6> &goal,
    const char *stage,
    double cart_linear_velocity)
{
    const std::filesystem::path file_path =
        project_root_dir() + "/picture_debug/line_trajectory_fail.txt";
    std::lock_guard<std::mutex> lock(g_line_traj_fail_log_mu);

    std::error_code ec;
    std::filesystem::create_directories(file_path.parent_path(), ec);
    if (ec)
    {
        std::cerr << "[line_trajectory_fail] 创建目录失败: " << ec.message() << std::endl;
        return;
    }

    std::ofstream out(file_path, std::ios::app);
    if (!out)
    {
        std::cerr << "[line_trajectory_fail] 无法写入 " << file_path.string() << std::endl;
        return;
    }

    const auto now = std::chrono::system_clock::now();
    const std::time_t sec = std::chrono::system_clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) %
                    1000;
    std::tm tm_local{};
    localtime_r(&sec, &tm_local);

    out << kLineTrajFailDebugSep;
    out << "[line_trajectory_fail] hand=" << (hand_label ? hand_label : "?")
        << " ret=" << ret << "\n";
    out << "stage=" << (stage && stage[0] != '\0' ? stage : "(未标注)") << "\n";
    out << "time=" << std::put_time(&tm_local, "%Y-%m-%d %H:%M:%S") << '.'
        << std::setw(3) << std::setfill('0') << ms.count() << "\n";
    out << "format: x,y,z(m) rx,ry,rz(rad)\n";
    out << "goal=" << pose6_to_csv(goal) << "\n";
    out << "cart_vel=" << std::fixed << std::setprecision(3) << cart_linear_velocity << "\n";
    out << kLineTrajFailDebugSep << "\n";
    out.flush();

    std::cerr << "[line_trajectory_fail] 已写入 " << file_path.string()
              << " hand=" << (hand_label ? hand_label : "?") << " ret=" << ret << std::endl;
}

namespace
{

struct T170Mdh
{
    double alpha[7]{};
    double a[7]{};
    double d[7]{};
    double dh_off[7]{};
    double mdl_dir[7]{};
    double mdl_off[7]{};
    Eigen::Matrix<double, 1, 6> tool = Eigen::Matrix<double, 1, 6>::Zero();
};

T170Mdh t170_mdh_from_yaml(bool is_right)
{
    const double h = M_PI / 2.0;
    T170Mdh m;
    const double al[7] = {h, h, h, -h, h, -h, -h};
    const double dho[7] = {h, -h, h, 0, 0, -h, 0};
    for (int i = 0; i < 7; ++i)
    {
        m.alpha[i] = al[i];
        m.a[i] = 0.0;
        m.dh_off[i] = dho[i];
        m.mdl_dir[i] = 1.0;
        m.mdl_off[i] = 0.0;
    }
    m.d[0] = is_right ? 0.191 : -0.191;
    m.d[2] = 0.270;
    m.d[4] = 0.250;
    if (is_right)
        m.mdl_off[1] = h;
    else
    {
        m.mdl_dir[0] = -1.0;
        m.mdl_dir[3] = -1.0;
        m.mdl_dir[5] = -1.0;
        m.mdl_off[1] = -h;
    }
    m.tool << 0.260, 0, 0, 0, 0, 0;
    return m;
}

double t170_q_model(const T170Mdh &m, int i, double q_motor, int conv)
{
    if (conv == 1)
        return q_motor * m.mdl_dir[i] + m.mdl_off[i];
    const double d = m.mdl_dir[i];
    return (q_motor - m.mdl_off[i]) / (std::abs(d) < 1e-9 ? 1.0 : d);
}

Eigen::Matrix4d t170_accum(const T170Mdh &m, const Eigen::RowVectorXd &q, int conv, int n_joints)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    const int n = std::min(n_joints, 7);
    for (int i = 0; i < n; ++i)
    {
        const double th = t170_q_model(m, i, q(i), conv) + m.dh_off[i];
        T *= MDHTrans(m.alpha[i], m.a[i], m.d[i], th);
    }
    return T;
}

double t170_tcp_err(Robot_Arm &arm, const T170Mdh &m, const Eigen::RowVectorXd &q, int conv)
{
    const Eigen::Vector3d sdk = arm.Forward_Kinematics(q).block<3, 1>(0, 3);
    const Eigen::Matrix4d T = t170_accum(m, q, conv, 7) * TR(m.tool);
    return (T.block<3, 1>(0, 3) - sdk).norm();
}

struct T170MdhPick
{
    T170Mdh m;
    int conv = 0;
    double err = 1e9;
};

T170MdhPick t170_pick_mdh(Robot_Arm &arm, const Eigen::RowVectorXd &q)
{
    const bool is_right = (arm_hand_side(arm) == 1);
    const T170Mdh cand = t170_mdh_from_yaml(is_right);
    T170MdhPick best;
    best.m = cand;
    for (int conv = 0; conv < 2; ++conv)
    {
        const double err = t170_tcp_err(arm, cand, q, conv);
        if (err < best.err)
        {
            best.conv = conv;
            best.err = err;
        }
    }
    static bool logged[2] = {false, false};
    const int side = is_right ? 1 : 0;
    if (!logged[side])
    {
        logged[side] = true;
        std::cout << "[fk] " << (is_right ? "右臂" : "左臂")
                  << " MDH conv=" << best.conv << " tcp_err=" << std::fixed << std::setprecision(5)
                  << best.err << " m（肘/腕用 yaml MDH 累乘，TCP 用 SDK）\n";
        if (best.err > 0.02)
            std::cerr << "[fk][WARN] 本地 MDH TCP 与 SDK 差 " << best.err
                      << " m，肘/腕坐标可能偏\n";
    }
    return best;
}

ArmLinkFk compute_link_fk(Robot_Arm &arm, const Eigen::RowVectorXd &q)
{
    ArmLinkFk out;
    out.tcp = arm.Forward_Kinematics(q).block<3, 1>(0, 3);
    const T170MdhPick pick = t170_pick_mdh(arm, q);
    out.tcp_mdh_err = pick.err;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 7; ++i)
    {
        const double th = t170_q_model(pick.m, i, q(i), pick.conv) + pick.m.dh_off[i];
        T *= MDHTrans(pick.m.alpha[i], pick.m.a[i], pick.m.d[i], th);
        const Eigen::Vector3d p = T.block<3, 1>(0, 3);
        if (i == 1)
            out.shoulder = p;
        else if (i == 3)
            out.elbow = p;
        else if (i == 4)
            out.wrist = p;
        else if (i == 6)
            out.flange = p;
    }
    return out;
}

} // namespace

ArmLinkFk arm_fk_links(Robot_Arm &arm, const Eigen::Ref<const Eigen::RowVectorXd> &q_motor)
{
    return compute_link_fk(arm, q_motor);
}

ArmLinkFk arm_get_link_fk(Robot_Arm &arm)
{
    const int hand = arm_hand_side(arm);
    return arm_fk_links(arm, arm_read_motor_joints(hand).row(0));
}

void log_arm_link_fk(Robot_Arm &arm, const char *tag)
{
    const ArmLinkFk L = arm_get_link_fk(arm);
    const int hand = arm_hand_side(arm);
    std::cout << std::fixed << std::setprecision(4)
              << "[fk] " << (tag ? tag : "") << " " << (hand == 1 ? "右臂" : "左臂")
              << " elbow=(" << L.elbow.x() << "," << L.elbow.y() << "," << L.elbow.z()
              << ") wrist=(" << L.wrist.x() << "," << L.wrist.y() << "," << L.wrist.z()
              << ") tcp=(" << L.tcp.x() << "," << L.tcp.y() << "," << L.tcp.z()
              << ") mdh_err=" << L.tcp_mdh_err << " m\n";
}

int arm_line_move(
    Robot_Arm &arm, Matrix<double, 1, 6> &pos, double cart_linear_velocity, bool snap_reach)
{
    if (arm_hand_side(arm) == 1 && right_arm_motors_locked())
        return 0;
    clamp_cart_goal_z(pos, "arm_line_move(直线运动下发前)");
    const int hand = arm_hand_side(arm);
    Eigen::MatrixXd q_start = arm_read_motor_joints(hand);
    if (hardware_abort_requested())
        return -4;
    if (can_io_faulted())
        return -2;

    arm.Cart_Linear_Velocity = cart_linear_velocity;

    Eigen::MatrixXd traj;
    const int ret = arm.Line_Trajectory(q_start, pos, traj);
    if (ret != 0)
    {
        const char *stage = nullptr;
        {
            std::lock_guard<std::mutex> lock(g_arm_line_move_stage_mu);
            stage = g_arm_line_move_debug_stage;
        }
        append_line_trajectory_fail_debug(arm_hand_label(hand), ret, pos, stage, cart_linear_velocity);
        return ret;
    }
    const int exec = execute_arm_trajectory(traj, hand, k_arm_traj_dt);
    if (exec == 0)
        record_arm_cartesian_arrival(arm, pos, nullptr);
    return exec;
}

static std::mutex g_arm_kin_mu[2];

namespace
{

constexpr double kServoDtS = 5e-3;
constexpr double kServoDeadM = 0.002;
constexpr double kServoVmaxMps = 0.05;
constexpr double kServoKp = 2.0;
constexpr double kServoAmax = 0.40;
/** 每拍最多走这么多。更大的解按这个比例缩了再发，不再整拍丢掉。 */
constexpr double kServoMaxJointStepRad = 0.02;
/** 超过这个值当成换了一组解，这一拍不跟。 */
constexpr double kServoBranchRad = 0.35;

double servo_apply_pose(Robot_Arm &arm, int hand, const Eigen::Matrix<double, 1, 6> &pose,
                        Eigen::RowVectorXd &q, int &ik_code, double &max_dq)
{
    ik_code = -1;
    max_dq = 0.0;
    if (q.size() < 7)
        return 0.0;
    Eigen::RowVectorXd q_try = q;
    auto joint_jump = [&](const Eigen::RowVectorXd &cand) {
        double jump = 0.0;
        for (int j = 0; j < 7; ++j)
            jump = std::max(jump, std::abs(cand(j) - q(j)));
        return jump;
    };
    auto usable = [&](const Eigen::RowVectorXd &cand, double &jump) {
        if (cand.size() < 7 || !cand.allFinite() || arm.Check_Joint_Limit(cand))
            return false;
        jump = joint_jump(cand);
        return jump <= kServoBranchRad;
    };
    {
        std::lock_guard<std::mutex> kin(g_arm_kin_mu[hand == 1 ? 1 : 0]);
        ik_code = arm.Inverse_Kinematics_Analytic(pose, q_try, q(1));
        double jump = 0.0;
        if (ik_code != 0 || !usable(q_try, jump))
        {
            q_try = q;
            Eigen::Matrix<double, 1, 6> pose_num = pose;
            const int numeric_rc = arm.Inverse_Kinematics_Numeric(pose_num, q_try);
            if (numeric_rc != 0 || !usable(q_try, jump))
            {
                max_dq = jump;
                if (ik_code == 0)
                    ik_code = numeric_rc != 0 ? numeric_rc : -12;
                return 0.0;
            }
            ik_code = 0;
        }
        max_dq = jump;
    }
    double scale = 1.0;
    if (max_dq > kServoMaxJointStepRad)
    {
        scale = kServoMaxJointStepRad / max_dq;
        q_try = q + scale * (q_try - q);
    }
    int motor_cmd[7] = {0};
    for (int j = 0; j < 7; ++j)
        motor_cmd[j] = rad_to_encoder_cnt(q_try(j));
    set_motor_position(motor_cmd, hand);
    save_arm_last_commanded_position(hand, motor_cmd);
    double q_note[7] = {};
    for (int j = 0; j < 7; ++j)
        q_note[j] = q_try(j);
    hand_shadow_note_q(hand, q_note);
    q = std::move(q_try);
    return scale;
}

struct ServoSide
{
    bool on = false;
    int hand = 0;
    Eigen::Matrix<double, 1, 6> cmd = Eigen::Matrix<double, 1, 6>::Zero();
    Eigen::Matrix<double, 1, 6> goal = Eigen::Matrix<double, 1, 6>::Zero();
    Eigen::Vector2d vel = Eigen::Vector2d::Zero();
    Eigen::RowVectorXd q;
    int ik_fail = 0;
    int clamped = 0;
    bool logged_fail = false;
};

void servo_init_side(ServoSide &side, Robot_Arm &arm, bool on,
                     const Eigen::Matrix<double, 1, 6> &start_pose)
{
    side.on = on && !(arm_hand_side(arm) == 1 && right_arm_motors_locked());
    if (!side.on)
        return;
    side.hand = arm_hand_side(arm);
    side.q.resize(7);
    if (side.hand >= 0 && side.hand <= 1 && g_arm_last_cmd_valid[side.hand])
    {
        for (int j = 0; j < 7; ++j)
            side.q(j) = encoder_cnt_to_rad(g_arm_last_cmd_cnt[side.hand][j]);
    }
    else
    {
        side.q = arm_read_motor_joints(side.hand).row(0);
    }
    side.cmd = start_pose;
    if (!side.cmd.allFinite() || side.q.size() < 7)
    {
        side.on = false;
        std::cerr << "[伺服] " << (side.hand == 1 ? "右手" : "左手")
                  << " 当前位姿无效，这一侧不跟踪\n";
        return;
    }
    double q_note[7] = {};
    for (int j = 0; j < 7; ++j)
        q_note[j] = side.q(j);
    hand_shadow_note_q(side.hand, q_note);
    side.goal = side.cmd;
}

void servo_step_side(ServoSide &side, Robot_Arm &arm)
{
    if (!side.on)
        return;
    const Eigen::Vector2d err(side.goal(0) - side.cmd(0), side.goal(1) - side.cmd(1));
    const double dist = err.norm();
    Eigen::Vector2d v_des = Eigen::Vector2d::Zero();
    if (dist > kServoDeadM)
    {
        const double vm = std::min(kServoVmaxMps, dist * kServoKp);
        v_des = err / dist * vm;
    }
    Eigen::Vector2d dv = v_des - side.vel;
    const double dvn = dv.norm();
    const double cap = kServoAmax * kServoDtS;
    if (dvn > cap && dvn > 1e-12)
        dv *= cap / dvn;
    const Eigen::Vector2d vel_next = side.vel + dv;
    Eigen::Matrix<double, 1, 6> next = side.cmd;
    next(0) += vel_next(0) * kServoDtS;
    next(1) += vel_next(1) * kServoDtS;
    int ik_code = 0;
    double max_dq = 0.0;
    const double scale = servo_apply_pose(arm, side.hand, next, side.q, ik_code, max_dq);
    if (scale > 0.0)
    {
        side.cmd(0) += (next(0) - side.cmd(0)) * scale;
        side.cmd(1) += (next(1) - side.cmd(1)) * scale;
        side.vel = vel_next * scale;
        if (scale < 0.999)
            ++side.clamped;
    }
    else
    {
        side.vel *= 0.5;
        ++side.ik_fail;
        if (!side.logged_fail)
        {
            side.logged_fail = true;
            std::cerr << std::fixed << std::setprecision(3)
                      << "[伺服] " << (side.hand == 1 ? "右手" : "左手")
                      << " 这一拍没发出去，逆解 code=" << ik_code
                      << " 关节差 " << (max_dq * 180.0 / M_PI) << "°\n";
        }
    }
}

} // namespace

int arm_dual_cartesian_track(
    Robot_Arm &arm_r,
    Robot_Arm &arm_l,
    const Eigen::Matrix<double, 1, 6> &start_r,
    const Eigen::Matrix<double, 1, 6> &start_l,
    bool move_r,
    bool move_l,
    double duration_s,
    const std::function<void(double t_s,
                             Eigen::Matrix<double, 1, 6> &goal_r,
                             Eigen::Matrix<double, 1, 6> &goal_l)> &update_goal)
{
    const bool until_abort = duration_s == 0.0;
    if ((!until_abort && !(duration_s > 0.0)) || !update_goal)
        return -1;
    ServoSide right;
    ServoSide left;
    servo_init_side(right, arm_r, move_r, start_r);
    servo_init_side(left, arm_l, move_l, start_l);
    if ((move_r && !right.on) || (move_l && !left.on))
        return -1;
    if (!right.on && !left.on)
        return 0;

    std::cout << std::fixed << std::setprecision(1)
              << "[伺服] 连续跟踪 ";
    if (until_abort)
        std::cout << "直到 abort";
    else
        std::cout << duration_s << " s";
    std::cout << "，每 5 ms 一步，"
              << "速度上限 " << (kServoVmaxMps * 1000.0)
              << " mm/s，加速度上限 " << kServoAmax
              << " m/s²，死区 " << (kServoDeadM * 1000.0) << " mm\n";

    const auto t0 = std::chrono::steady_clock::now();
    int ticks = 0;
    while (true)
    {
        if (hardware_abort_requested())
            return -4;
        if (can_io_faulted())
            return -2;
        const auto cycle = std::chrono::steady_clock::now();
        const double t = std::chrono::duration<double>(cycle - t0).count();
        if (!until_abort && t >= duration_s)
            break;
        update_goal(t, right.goal, left.goal);
        servo_step_side(right, arm_r);
        servo_step_side(left, arm_l);
        if (ticks % 20 == 0)
        {
            auto report = [](const char *name, const ServoSide &side) {
                if (!side.on)
                    return;
                const double ex = (side.goal(0) - side.cmd(0)) * 1000.0;
                const double ey = (side.goal(1) - side.cmd(1)) * 1000.0;
                const double sp = side.vel.norm() * 1000.0;
                std::cout << std::fixed << std::setprecision(1)
                          << "  " << name << " 还差 " << std::hypot(ex, ey)
                          << " mm  速度 " << sp << " mm/s";
            };
            std::cout << "[伺服] t=" << std::setprecision(1) << t << "s";
            report("右", right);
            report("左", left);
            std::cout << std::endl;
        }
        if (ticks % 200 == 0)
        {
            auto one = [](const char *name, const ServoSide &side) {
                if (!side.on)
                    return std::string();
                const double left_mm = std::hypot((side.goal(0) - side.cmd(0)) * 1000.0,
                                                  (side.goal(1) - side.cmd(1)) * 1000.0);
                std::ostringstream text;
                text << std::fixed << std::setprecision(1) << name << " 还差 " << left_mm
                     << " mm 失败 " << side.ik_fail << " 拍";
                return text.str();
            };
            const std::string right_text = one("右", right);
            const std::string left_text = one("左", left);
            std::string detail;
            if (!right_text.empty())
                detail += right_text;
            if (!left_text.empty())
            {
                if (!detail.empty())
                    detail += "；";
                detail += left_text;
            }
            hand_track_log("伺服", "跟踪", detail);
        }
        ++ticks;
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - cycle).count();
        const double sleep_ms = std::max(0.0, kServoDtS * 1000.0 - elapsed_ms);
        if (sleep_ms > 1e-6)
            std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(sleep_ms));
    }
    std::cout << "[伺服] 跟踪结束"
              << " 右逆解失败 " << right.ik_fail << " 拍、限幅 " << right.clamped
              << " 拍；左逆解失败 " << left.ik_fail << " 拍、限幅 " << left.clamped
              << " 拍\n";
    return 0;
}

int arm_line_move_settle(Robot_Arm &arm, Matrix<double, 1, 6> &pos, double cart_linear_velocity, int settle_ms)
{
    if (arm_hand_side(arm) == 1 && right_arm_motors_locked())
        return 0;
    clamp_cart_goal_z(pos, "arm_line_move_settle(直线运动下发前)");
    const int hand = arm_hand_side(arm);
    Eigen::MatrixXd q_start = arm_read_motor_joints(hand);
    if (hardware_abort_requested())
        return -4;
    if (can_io_faulted())
        return -2;

    arm.Cart_Linear_Velocity = cart_linear_velocity;

    Eigen::MatrixXd traj;
    const int ret = arm.Line_Trajectory(q_start, pos, traj);
    if (ret != 0)
    {
        const char *stage = nullptr;
        {
            std::lock_guard<std::mutex> lock(g_arm_line_move_stage_mu);
            stage = g_arm_line_move_debug_stage;
        }
        append_line_trajectory_fail_debug(arm_hand_label(hand), ret, pos, stage, cart_linear_velocity);
        return ret;
    }
    const int exec = execute_arm_trajectory(traj, hand, k_arm_traj_dt);
    if (exec != 0)
        return exec;
    if (settle_ms > 0)
    {
        std::cout << "[arm] 轨迹已下发完，再等 " << settle_ms << " ms 后回读关节角\n" << std::flush;
        hardware_abort_sleep_ms(settle_ms);
    }
    if (hardware_abort_requested())
        return -4;

    const int n = static_cast<int>(std::min(traj.cols(), static_cast<Eigen::Index>(7)));
    const Eigen::Matrix<double, 1, 7> q_act = arm_get_joint_pos(arm);
    std::cout << std::fixed << std::setprecision(3)
              << "[arm] " << (hand == 1 ? "右手" : "左手")
              << " 关节角(度)  理论  实际  差\n";
    for (int j = 0; j < n; ++j)
    {
        const double theory_deg = traj(traj.rows() - 1, j) * 180.0 / M_PI;
        const double actual_deg = q_act(j) * 180.0 / M_PI;
        std::cout << "  J" << (j + 1)
                  << "  " << theory_deg
                  << "  " << actual_deg
                  << "  " << (actual_deg - theory_deg) << "\n";
    }
    const Eigen::Matrix<double, 1, 6> tcp = arm_get_tcp_pos(arm);
    std::cout << std::setprecision(4)
              << "[arm] 等稳后 TCP 目标=(" << pos(0) << "," << pos(1) << "," << pos(2)
              << ") 实际=(" << tcp(0) << "," << tcp(1) << "," << tcp(2)
              << ") 差=(" << (tcp(0) - pos(0)) << "," << (tcp(1) - pos(1)) << ","
              << (tcp(2) - pos(2)) << ")\n";
    record_arm_cartesian_arrival(arm, pos, nullptr);
    return 0;
}

void log_arm_encoder_command_vs_actual(Robot_Arm &arm, const char *tag)
{
    const int hand = arm_hand_side(arm);
    const char *name = (tag && tag[0]) ? tag : (hand == 1 ? "右手" : "左手");
    if (hand < 0 || hand > 1 || !g_arm_last_cmd_valid[hand])
    {
        std::cout << "[tray2] " << name << " 没有本次下发的编码器理论值\n" << std::flush;
        return;
    }
    int actual[7] = {};
    get_motor_position(actual, hand);
    std::cout << "[tray2] " << name << " 编码器  理论  实际  差\n";
    for (int i = 0; i < 7; ++i)
    {
        const int theory = g_arm_last_cmd_cnt[hand][i];
        std::cout << "  J" << (i + 1)
                  << "  " << theory
                  << "  " << actual[i]
                  << "  " << (actual[i] - theory) << "\n";
    }
    std::cout << std::flush;
}

Eigen::Matrix<double, 1, 7> arm_get_joint_pos(Robot_Arm &arm)
{
    const Eigen::MatrixXd q = arm_read_motor_joints(arm_hand_side(arm));
    Eigen::Matrix<double, 1, 7> out = Eigen::Matrix<double, 1, 7>::Zero();
    const int n = static_cast<int>(std::min(q.cols(), static_cast<Eigen::Index>(7)));
    for (int i = 0; i < n; ++i)
        out(i) = q(0, i);
    return out;
}

int arm_line_move_hold_redundant(Robot_Arm &arm, Matrix<double, 1, 6> &pos, double cart_linear_velocity)
{
    if (arm_hand_side(arm) == 1 && right_arm_motors_locked())
        return 0;
    clamp_cart_goal_z(pos, "arm_line_move_hold_redundant(直线运动下发前)");
    const int hand = arm_hand_side(arm);
    Eigen::MatrixXd q_start = arm_read_motor_joints(hand);
    if (hardware_abort_requested())
        return -4;
    if (can_io_faulted())
        return -2;

    arm.Cart_Linear_Velocity = cart_linear_velocity;

    Eigen::MatrixXd traj;
    const double j2 = (q_start.cols() >= 2) ? q_start(0, 1) : 0.0;
    const int ret = arm.Line_Trajectory(q_start, pos, traj, j2);
    if (ret != 0)
    {
        const char *stage = nullptr;
        {
            std::lock_guard<std::mutex> lock(g_arm_line_move_stage_mu);
            stage = g_arm_line_move_debug_stage;
        }
        append_line_trajectory_fail_debug(arm_hand_label(hand), ret, pos, stage, cart_linear_velocity);
        return ret;
    }
    const int exec = execute_arm_trajectory(traj, hand, k_arm_traj_dt);
    if (exec == 0)
        record_arm_cartesian_arrival(arm, pos, nullptr);
    return exec;
}

ArmLineMoveResult arm_dual_line_move_hold_redundant_selective(
    Robot_Arm &arm_r,
    Matrix<double, 1, 6> &pos_r,
    const bool move_r,
    Robot_Arm &arm_l,
    Matrix<double, 1, 6> &pos_l,
    const bool move_l,
    const double cart_linear_velocity)
{
    ArmLineMoveResult out;
    if (move_r && move_l)
    {
        std::thread th_r([&]() {
            out.ret_r = arm_line_move_hold_redundant(arm_r, pos_r, cart_linear_velocity);
        });
        std::thread th_l([&]() {
            out.ret_l = arm_line_move_hold_redundant(arm_l, pos_l, cart_linear_velocity);
        });
        th_r.join();
        th_l.join();
        return out;
    }
    if (move_r)
        out.ret_r = arm_line_move_hold_redundant(arm_r, pos_r, cart_linear_velocity);
    if (move_l)
        out.ret_l = arm_line_move_hold_redundant(arm_l, pos_l, cart_linear_velocity);
    return out;
}

ArmLineMoveResult arm_dual_line_move(
    Robot_Arm &arm_r,
    Matrix<double, 1, 6> &pos_r,
    Robot_Arm &arm_l,
    Matrix<double, 1, 6> &pos_l,
    double cart_linear_velocity,
    bool snap_reach)
{
    ArmLineMoveResult out;
    std::thread th_r([&]() { out.ret_r = arm_line_move(arm_r, pos_r, cart_linear_velocity, snap_reach); });
    std::thread th_l([&]() { out.ret_l = arm_line_move(arm_l, pos_l, cart_linear_velocity, snap_reach); });
    th_r.join();
    th_l.join();
    return out;
}

ArmLineMoveResult arm_dual_line_move_selective(
    Robot_Arm &arm_r,
    Matrix<double, 1, 6> &pos_r,
    const bool move_r,
    Robot_Arm &arm_l,
    Matrix<double, 1, 6> &pos_l,
    const bool move_l,
    const double cart_linear_velocity,
    const bool snap_reach)
{
    if (move_r && move_l)
        return arm_dual_line_move(arm_r, pos_r, arm_l, pos_l, cart_linear_velocity, snap_reach);

    ArmLineMoveResult out;
    if (move_r)
        out.ret_r = arm_line_move(arm_r, pos_r, cart_linear_velocity, snap_reach);
    if (move_l)
        out.ret_l = arm_line_move(arm_l, pos_l, cart_linear_velocity, snap_reach);
    return out;
}

int g_planned_descent_row[2] = {-1, -1};
std::atomic<int> g_descent_fail_hand{-1};

namespace
{


constexpr double kCornerBlendM = 0.06;
constexpr double kCornerMaxStepRad = 8.0 * M_PI / 180.0;
constexpr double kCornerCruiseFrac = 0.65;

Eigen::Vector3d corner_fk_xyz(Robot_Arm &arm, const Eigen::Ref<const Eigen::RowVectorXd> &q)
{
    return arm.Forward_Kinematics(q).block<3, 1>(0, 3);
}

int plan_rounded_corner(
    Robot_Arm &arm,
    Eigen::Matrix<double, 1, 6> hover,
    Eigen::Matrix<double, 1, 6> final_pose,
    double cruise_mps,
    Eigen::MatrixXd &traj_out)
{
    traj_out.resize(0, 0);
    const int hand = arm_hand_side(arm);
    if (hand == 1 && right_arm_motors_locked())
        return 0;
    if (hardware_abort_requested())
        return -4;
    if (can_io_faulted())
        return -2;
    if (!hover.allFinite() || !final_pose.allFinite() || !(cruise_mps > 0.0))
        return -10;

    clamp_cart_goal_z(hover, "抹角B");
    clamp_cart_goal_z(final_pose, "抹角C");
    Eigen::MatrixXd q_start = arm_read_motor_joints(hand);
    if (q_start.rows() != 1 || q_start.cols() < 6 || !q_start.allFinite())
        return -10;

    const double cruise = std::max(0.02, cruise_mps);
    arm.Cart_Linear_Velocity = cruise;
    Eigen::Matrix<double, 1, 6> goal_b = hover;
    Eigen::MatrixXd traj_ab;
    const int rc_ab = arm.Line_Trajectory(q_start, goal_b, traj_ab);
    if (rc_ab != 0 || traj_ab.rows() < 2)
    {
        append_line_trajectory_fail_debug(
            arm_hand_label(hand), rc_ab != 0 ? rc_ab : -16, hover, "抹角 A→B", cruise);
        return rc_ab != 0 ? rc_ab : -16;
    }

    Eigen::MatrixXd q_b(1, traj_ab.cols());
    q_b.row(0) = traj_ab.row(traj_ab.rows() - 1);
    Eigen::Matrix<double, 1, 6> goal_c = final_pose;
    Eigen::MatrixXd traj_bc;
    const int rc_bc = arm.Line_Trajectory(q_b, goal_c, traj_bc);
    if (rc_bc != 0 || traj_bc.rows() < 2 || traj_bc.cols() != traj_ab.cols())
    {
        append_line_trajectory_fail_debug(
            arm_hand_label(hand), rc_bc != 0 ? rc_bc : -16, final_pose, "抹角 B→C", cruise);
        return rc_bc != 0 ? rc_bc : -16;
    }

    const int n_ab = static_cast<int>(traj_ab.rows());
    const Eigen::Vector3d b_xyz(hover(0), hover(1), hover(2));
    const Eigen::Vector3d c_xyz(final_pose(0), final_pose(1), final_pose(2));
    std::vector<Eigen::Vector3d> tail_p(static_cast<size_t>(n_ab));
    std::vector<char> have(static_cast<size_t>(n_ab), 0);
    int fk_count = 0;
    auto pos_at = [&](int i) -> const Eigen::Vector3d & {
        if (!have[static_cast<size_t>(i)])
        {
            tail_p[static_cast<size_t>(i)] = corner_fk_xyz(arm, traj_ab.row(i));
            have[static_cast<size_t>(i)] = 1;
            ++fk_count;
        }
        return tail_p[static_cast<size_t>(i)];
    };

    int leave = 0;
    int fast = 0;
    bool got_leave = false;
    double back = 0.0;
    for (int i = n_ab - 1; i > 0; --i)
    {
        const double seg = (pos_at(i) - pos_at(i - 1)).norm();
        const double spd = seg / k_arm_traj_dt;
        back += seg;
        if (!got_leave && back >= kCornerBlendM)
        {
            leave = i - 1;
            got_leave = true;
            if (spd >= kCornerCruiseFrac * cruise)
            {
                fast = leave;
                break;
            }
        }
        if (got_leave && spd >= kCornerCruiseFrac * cruise)
        {
            fast = i - 1;
            break;
        }
        if (got_leave && back > kCornerBlendM + 0.12)
        {
            fast = i - 1;
            break;
        }
    }
    if (!got_leave)
    {
        leave = 0;
        fast = 0;
        (void)pos_at(0);
    }
    if (fast > leave)
        fast = leave;

    const Eigen::RowVectorXd q_c = traj_bc.row(traj_bc.rows() - 1);
    const double leave_to_b = (pos_at(leave) - b_xyz).norm();
    const double join_to_b = (c_xyz - b_xyz).norm();
    const double chord = std::max(1e-4, (pos_at(leave) - c_xyz).norm());

    std::vector<Eigen::RowVectorXd> rows;
    rows.reserve(static_cast<size_t>(fast + 8) + 160);
    for (int i = 0; i <= fast; ++i)
        rows.push_back(traj_ab.row(i));

    double tail_arc = 0.0;
    if (fast < leave)
    {
        std::vector<double> cum(static_cast<size_t>(leave - fast + 1), 0.0);
        for (int i = fast + 1; i <= leave; ++i)
        {
            tail_arc += (pos_at(i) - pos_at(i - 1)).norm();
            cum[static_cast<size_t>(i - fast)] = tail_arc;
        }
        const int n_tail = std::max(1, static_cast<int>(std::ceil(tail_arc / cruise / k_arm_traj_dt)));
        for (int k = 1; k <= n_tail; ++k)
        {
            const double s = tail_arc * (static_cast<double>(k) / static_cast<double>(n_tail));
            int seg = fast;
            while (seg < leave && cum[static_cast<size_t>(seg - fast + 1)] < s)
                ++seg;
            const double s0 = cum[static_cast<size_t>(seg - fast)];
            const double s1 = cum[static_cast<size_t>(std::min(seg + 1, leave) - fast)];
            const double den = std::max(1e-12, s1 - s0);
            const double t = std::clamp((s - s0) / den, 0.0, 1.0);
            const int i1 = std::min(seg + 1, leave);
            rows.push_back(traj_ab.row(seg) + t * (traj_ab.row(i1) - traj_ab.row(seg)));
        }
    }

    const double blend_T = std::max(0.20, 2.0 * chord / cruise);
    const int n_blend = std::max(2, static_cast<int>(std::ceil(blend_T / k_arm_traj_dt)));
    const Eigen::RowVectorXd q_leave = rows.back();
    for (int k = 1; k <= n_blend; ++k)
    {
        const double u = static_cast<double>(k) / static_cast<double>(n_blend);
        const double s = u * (2.0 - u);
        rows.push_back(q_leave + s * (q_c - q_leave));
    }

    const int dof = static_cast<int>(rows.front().cols());
    traj_out.resize(static_cast<Eigen::Index>(rows.size()), dof);
    traj_out.row(0) = rows.front();
    double max_step = 0.0;
    for (size_t i = 1; i < rows.size(); ++i)
    {
        traj_out.row(static_cast<Eigen::Index>(i)) = rows[i];
        max_step = std::max(
            max_step, (traj_out.row(static_cast<Eigen::Index>(i)) - traj_out.row(static_cast<Eigen::Index>(i - 1)))
                          .cwiseAbs()
                          .maxCoeff());
    }
    if (max_step > kCornerMaxStepRad)
    {
        std::cerr << std::fixed << std::setprecision(2)
                  << "[corner] " << arm_hand_label(hand)
                  << " 抹角后单步关节 " << (max_step * 180.0 / M_PI)
                  << "°，拒绝下发\n";
        traj_out.resize(0, 0);
        return -12;
    }

    const double prefix_s = static_cast<double>(fast) * k_arm_traj_dt;
    std::cout << std::fixed << std::setprecision(3)
              << "[corner] " << arm_hand_label(hand)
              << " 原样 " << prefix_s << " s 到切点，离开B " << leave_to_b
              << " m，接入距B " << join_to_b
              << " m，抹角 " << std::setprecision(2) << blend_T
              << " s，巡航 " << cruise
              << " m/s，规划点 AB=" << n_ab << " BC=" << traj_bc.rows()
              << " FK=" << fk_count << "\n";
    return 0;
}

double traj_joint_step(const Eigen::MatrixXd &traj, int i)
{
    return (traj.row(i) - traj.row(i - 1)).norm();
}

double traj_peak_step(const Eigen::MatrixXd &traj)
{
    double peak = 0.0;
    for (int i = 1; i < traj.rows(); ++i)
        peak = std::max(peak, traj_joint_step(traj, i));
    return peak;
}

int last_cruise_row(const Eigen::MatrixXd &traj)
{
    const int n = static_cast<int>(traj.rows());
    if (n < 3)
        return n - 1;
    const double th = std::max(1e-6, 0.55 * traj_peak_step(traj));
    int last = 1;
    for (int i = 1; i < n; ++i)
    {
        if (traj_joint_step(traj, i) >= th)
            last = i;
    }
    return last;
}

int first_cruise_row(const Eigen::MatrixXd &traj)
{
    const int n = static_cast<int>(traj.rows());
    if (n < 3)
        return 0;
    const double th = std::max(1e-6, 0.55 * traj_peak_step(traj));
    for (int i = 1; i < n; ++i)
    {
        if (traj_joint_step(traj, i) >= th)
            return i;
    }
    return n - 1;
}

int plan_arm_line_from(
    Robot_Arm &arm,
    Eigen::Matrix<double, 1, 6> goal,
    double vel_m_s,
    const Eigen::MatrixXd &q_start,
    Eigen::MatrixXd &traj)
{
    if (q_start.rows() != 1 || q_start.cols() < 6 || !q_start.allFinite() || !goal.allFinite())
        return -10;
    if (!(vel_m_s > 0.0))
        return -10;
    clamp_cart_goal_z(goal, "两段直线");
    arm.Cart_Linear_Velocity = vel_m_s;
    const int ret = arm.Line_Trajectory(q_start, goal, traj);
    if (ret != 0 || traj.rows() < 2)
        return ret != 0 ? ret : -16;
    return 0;
}

Eigen::MatrixXd stitch_cruise_lines(const Eigen::MatrixXd &first, const Eigen::MatrixXd &second)
{
    const int ia = last_cruise_row(first);
    const int ib = first_cruise_row(second);
    const Eigen::RowVectorXd qa = first.row(ia);
    const Eigen::RowVectorXd qb = second.row(ib);
    const int dof = static_cast<int>(qa.cols());
    // 只用抬升段的巡航步长。回程段里腕子的大步不能拿来缩短接缝，否则整条手臂会抽一下。
    double step = (ia >= 1) ? traj_joint_step(first, ia) : 1e-3;
    step = std::max(step, 1e-4);
    const double dist = (qa - qb).norm();
    int n_bridge = 1;
    if (std::isfinite(dist) && std::isfinite(step) && step > 0.0)
        n_bridge = std::max(1, static_cast<int>(std::ceil(dist / step)));
    const double max_joint = (qa - qb).cwiseAbs().maxCoeff();
    // 接缝单个关节不超过 25°/s。原先 8°/5ms 约合 1600°/s，腕子和收臂都会抽。
    const double max_step = 25.0 * M_PI / 180.0 * k_arm_traj_dt;
    if (std::isfinite(max_joint) && max_joint > max_step * static_cast<double>(n_bridge))
        n_bridge = std::max(1, static_cast<int>(std::ceil(max_joint / max_step)));
    if (n_bridge > 2000)
        n_bridge = 2000;

    const int n_tail = static_cast<int>(second.rows()) - ib;
    if (n_tail < 0 || ia < 0)
        return Eigen::MatrixXd();
    const int n = ia + n_bridge + n_tail;
    if (n <= 0 || n > 20000)
        return Eigen::MatrixXd();
    Eigen::MatrixXd out(n, dof);
    for (int i = 0; i <= ia; ++i)
        out.row(i) = first.row(i);
    for (int k = 1; k < n_bridge; ++k)
    {
        const double u = static_cast<double>(k) / static_cast<double>(n_bridge);
        out.row(ia + k) = qa + u * (qb - qa);
    }
    for (int j = 0; j < n_tail; ++j)
        out.row(ia + n_bridge + j) = second.row(ib + j);
    return out;
}

int plan_line_then_line(
    Robot_Arm &arm,
    Eigen::Matrix<double, 1, 6> mid,
    Eigen::Matrix<double, 1, 6> goal,
    double vel_first_m_s,
    double vel_second_m_s,
    Eigen::MatrixXd &traj_out,
    const char *tag)
{
    const int hand = arm_hand_side(arm);
    if (hand == 1 && right_arm_motors_locked())
        return 0;
    if (hand >= 0 && hand <= 1)
        g_planned_descent_row[hand] = -1;
    Eigen::MatrixXd q_start = arm_read_motor_joints(hand);
    Eigen::MatrixXd traj_mid;
    const int rc_mid = plan_arm_line_from(arm, mid, vel_first_m_s, q_start, traj_mid);
    if (rc_mid != 0)
        return rc_mid;
    Eigen::MatrixXd q_mid(1, traj_mid.cols());
    q_mid.row(0) = traj_mid.row(traj_mid.rows() - 1);
    Eigen::MatrixXd traj_goal;
    const int rc_goal = plan_arm_line_from(arm, goal, vel_second_m_s, q_mid, traj_goal);
    if (rc_goal != 0)
        return rc_goal;
    const int ib = first_cruise_row(traj_goal);
    const int n_tail = static_cast<int>(traj_goal.rows()) - ib;
    traj_out = stitch_cruise_lines(traj_mid, traj_goal);
    if (traj_out.rows() < 2)
        return -16;
    int descent = static_cast<int>(traj_out.rows()) - n_tail;
    if (descent < 1)
        descent = 1;
    if (descent >= traj_out.rows())
        descent = static_cast<int>(traj_out.rows()) - 1;
    if (hand >= 0 && hand <= 1)
        g_planned_descent_row[hand] = descent;
    std::cout << "[" << (tag != nullptr ? tag : "line") << "] " << arm_hand_label(hand)
              << " 两段直线中间不停"
              << " 点=" << traj_out.rows()
              << " 第一段=" << traj_mid.rows()
              << " 第二段=" << traj_goal.rows()
              << " 下降起行=" << descent << "\n";
    return 0;
}

struct PreparedLinePlan
{
    std::mutex mu;
    std::thread worker;
    bool have = false;
    int ret = -1;
    Eigen::MatrixXd traj;
};

PreparedLinePlan g_prepared_line[2];

bool take_prepared_line(int hand, Eigen::MatrixXd &traj, int &ret)
{
    if (hand < 0 || hand > 1)
        return false;
    PreparedLinePlan &slot = g_prepared_line[hand];
    if (slot.worker.joinable())
        slot.worker.join();
    std::lock_guard<std::mutex> lock(slot.mu);
    if (!slot.have || slot.traj.rows() < 2)
    {
        slot.have = false;
        return false;
    }
    const Eigen::MatrixXd q_now = arm_read_motor_joints(hand);
    const double mismatch =
        (q_now.cols() == slot.traj.cols())
            ? (q_now.row(0) - slot.traj.row(0)).cwiseAbs().maxCoeff()
            : 1.0;
    if (!(mismatch <= 0.05))
    {
        std::cout << "[retract] " << arm_hand_label(hand)
                  << " 预规划起点已偏离，重新规划\n";
        slot.have = false;
        slot.traj.resize(0, 0);
        return false;
    }
    traj = slot.traj;
    ret = slot.ret;
    slot.have = false;
    slot.traj.resize(0, 0);
    std::cout << "[retract] " << arm_hand_label(hand)
              << " 合爪期间已规划好，直接下发\n";
    return true;
}

} // namespace

int arm_reorient_limited(Robot_Arm &arm, Eigen::Matrix<double, 1, 6> goal, double max_deg_s)
{
    const int hand = arm_hand_side(arm);
    if (hand == 1 && right_arm_motors_locked())
        return 0;
    if (!(max_deg_s > 1.0))
        max_deg_s = 40.0;
    const Eigen::MatrixXd q_start = arm_read_motor_joints(hand);
    if (q_start.rows() != 1 || q_start.cols() < 6 || !q_start.allFinite() || !goal.allFinite())
        return -10;
    Eigen::MatrixXd planned;
    arm.Cart_Linear_Velocity = 0.05;
    const int ret = arm.Line_Trajectory(q_start, goal, planned);
    if (ret != 0 || planned.rows() < 2 || planned.cols() != q_start.cols())
        return ret != 0 ? ret : -16;
    const Eigen::RowVectorXd q_end = planned.row(planned.rows() - 1);
    const double max_joint = (q_end - q_start.row(0)).cwiseAbs().maxCoeff();
    const double max_step = max_deg_s * M_PI / 180.0 * k_arm_traj_dt;
    int n = std::max(2, static_cast<int>(std::ceil(max_joint / std::max(max_step, 1e-6))));
    if (n > 8000)
        n = 8000;
    Eigen::MatrixXd traj(n + 1, q_start.cols());
    for (int i = 0; i <= n; ++i)
    {
        const double u = static_cast<double>(i) / static_cast<double>(n);
        traj.row(i) = q_start.row(0) + u * (q_end - q_start.row(0));
    }
    return execute_arm_trajectory(traj, hand, k_arm_traj_dt);
}

void arm_start_line_then_line_plan(
    Robot_Arm &arm,
    const Eigen::Matrix<double, 1, 6> &mid,
    const Eigen::Matrix<double, 1, 6> &goal,
    const double vel_first_m_s,
    const double vel_second_m_s)
{
    const int hand = arm_hand_side(arm);
    if (hand < 0 || hand > 1)
        return;
    if (hand == 1 && right_arm_motors_locked())
        return;
    PreparedLinePlan &slot = g_prepared_line[hand];
    if (slot.worker.joinable())
        slot.worker.join();
    {
        std::lock_guard<std::mutex> lock(slot.mu);
        slot.have = false;
        slot.ret = -1;
        slot.traj.resize(0, 0);
    }
    Robot_Arm *arm_ptr = &arm;
    const Eigen::Matrix<double, 1, 6> mid_copy = mid;
    const Eigen::Matrix<double, 1, 6> goal_copy = goal;
    slot.worker = std::thread([arm_ptr, hand, mid_copy, goal_copy, vel_first_m_s, vel_second_m_s]() {
        int rc = -16;
        Eigen::MatrixXd traj;
        try
        {
            rc = plan_line_then_line(
                *arm_ptr, mid_copy, goal_copy, vel_first_m_s, vel_second_m_s, traj, "收手");
        }
        catch (const std::exception &ex)
        {
            std::cerr << "[retract] 预规划失败: " << ex.what() << "\n";
            rc = -16;
        }
        std::lock_guard<std::mutex> lock(g_prepared_line[hand].mu);
        g_prepared_line[hand].ret = rc;
        g_prepared_line[hand].have = rc == 0 && traj.rows() >= 2;
        if (g_prepared_line[hand].have)
            g_prepared_line[hand].traj = std::move(traj);
    });
}

namespace
{

struct DescentSlot
{
    std::mutex mu;
    bool active = false;
    bool in_descent = false;
    bool ik_failed = false;
    int cursor = 0;
    int descent_row = 0;
    int plan_rows = 0;
    double applied_x = 0.0;
    double applied_y = 0.0;
    bool have_visual_xy = false;
    double bias_x = 0.0;
    double bias_y = 0.0;
    /** 框中心离夹爪的像素。小于 0 表示这一帧没有锁定。 */
    double align_px = -1.0;
    Eigen::Matrix<double, 1, 6> goal = Eigen::Matrix<double, 1, 6>::Zero();
    Robot_Arm *arm = nullptr;
    Eigen::MatrixXd traj;
    Eigen::MatrixXd base;
};

DescentSlot g_desc[2];

const char *descent_hand_name(int hand)
{
    return hand == 1 ? "右手" : "左手";
}

Eigen::Vector3d tcp_translation(Robot_Arm &arm, const Eigen::RowVectorXd &q)
{
    return arm.Forward_Kinematics(q).block<3, 1>(0, 3);
}

/** 在已经走通的关节上，求一个最小的关节增量，只动末端水平位置。阻尼最小二乘，不会另起一条逆解分支。 */
bool tcp_xy_joint_step(
    Robot_Arm &arm,
    const Eigen::RowVectorXd &q_anchor,
    const Eigen::RowVectorXd &q_last,
    double dx,
    double dy,
    Eigen::RowVectorXd &dq_out)
{
    dq_out.resize(0);
    if (q_anchor.size() < 7 || q_last.size() != q_anchor.size() || !q_anchor.allFinite() ||
        !q_last.allFinite() || !std::isfinite(dx) || !std::isfinite(dy))
        return false;
    const int dof = static_cast<int>(q_anchor.size());
    const int side = arm_hand_side(arm) == 1 ? 1 : 0;
    std::lock_guard<std::mutex> kin(g_arm_kin_mu[side]);
    const Eigen::Vector3d p0 = tcp_translation(arm, q_anchor);
    if (!p0.allFinite())
        return false;

    Eigen::Vector3d target(dx, dy, 0.0);
    Eigen::RowVectorXd q = q_anchor;
    constexpr double h = 1e-4;
    constexpr double lambda2 = 1e-4;
    for (int iter = 0; iter < 4; ++iter)
    {
        const Eigen::Vector3d p = tcp_translation(arm, q);
        if (!p.allFinite())
            return false;
        const Eigen::Vector3d err = target - (p - p0);
        if (err.norm() < 2e-4)
            break;
        Eigen::MatrixXd J(3, dof);
        bool jac_ok = true;
        for (int j = 0; j < dof; ++j)
        {
            Eigen::RowVectorXd qp = q;
            qp(j) += h;
            const Eigen::Vector3d pp = tcp_translation(arm, qp);
            if (!pp.allFinite())
            {
                jac_ok = false;
                break;
            }
            J.col(j) = (pp - p) / h;
        }
        if (!jac_ok || !J.allFinite())
            return false;
        Eigen::Matrix3d A = J * J.transpose();
        A.diagonal().array() += lambda2;
        const Eigen::Vector3d y = A.ldlt().solve(err);
        if (!y.allFinite())
            return false;
        Eigen::RowVectorXd step = (J.transpose() * y).transpose();
        const double max_abs = step.cwiseAbs().maxCoeff();
        constexpr double kMaxStep = 0.12;
        if (max_abs > kMaxStep && max_abs > 1e-12)
            step *= kMaxStep / max_abs;

        Eigen::RowVectorXd q_try = q + step;
        double scale = 1.0;
        auto step_fits = [&](double s) {
            const Eigen::RowVectorXd qn = q + s * step;
            return qn.allFinite() && !arm.Check_Joint_Limit(qn);
        };
        while (scale > 0.02 && !step_fits(scale))
            scale *= 0.5;
        if (!step_fits(scale))
            break;
        q = q + scale * step;
    }
    (void)q_last;
    dq_out = q - q_anchor;
    return dq_out.size() == dof && dq_out.allFinite();
}

bool tcp_xy_of(Robot_Arm &arm, const Eigen::RowVectorXd &q, double &x, double &y)
{
    if (q.size() < 7 || !q.allFinite())
        return false;
    const int side = arm_hand_side(arm) == 1 ? 1 : 0;
    std::lock_guard<std::mutex> kin(g_arm_kin_mu[side]);
    const Eigen::Vector3d p = tcp_translation(arm, q);
    if (!p.allFinite())
        return false;
    x = p.x();
    y = p.y();
    return true;
}

/** 未下发的行改回「当前点沿原下降走，再加上这一帧的水平增量」。旧的未下发修正被盖掉。 */
double rewrite_unsent_xy(
    Eigen::MatrixXd &traj,
    const Eigen::MatrixXd &base,
    int anchor,
    int from,
    const Eigen::RowVectorXd &dq)
{
    if (anchor < 0 || from <= anchor || from >= traj.rows() || base.rows() <= anchor ||
        base.cols() != traj.cols() || dq.size() != traj.cols() || !dq.allFinite())
        return 0.0;
    const double max_joint = dq.cwiseAbs().maxCoeff();
    const double max_step = 25.0 * M_PI / 180.0 * k_arm_traj_dt;
    int ramp = 1;
    if (max_step > 1e-9 && max_joint > max_step)
        ramp = std::min(400, static_cast<int>(std::ceil(max_joint / max_step)));
    const int remain = static_cast<int>(traj.rows()) - from;
    if (remain < 1)
        return 0.0;
    double fit = 1.0;
    if (ramp > remain && ramp > 0)
    {
        fit = static_cast<double>(remain) / static_cast<double>(ramp);
        ramp = remain;
    }
    if (ramp < 1)
        ramp = 1;
    const Eigen::RowVectorXd use = fit * dq;
    const Eigen::RowVectorXd q_anchor = traj.row(anchor);
    const Eigen::RowVectorXd base_anchor = base.row(anchor);
    for (int r = from; r < traj.rows(); ++r)
    {
        const int k = r - from;
        const double u = std::min(1.0, static_cast<double>(k + 1) / static_cast<double>(ramp));
        Eigen::RowVectorXd hold = q_anchor;
        if (r < base.rows())
            hold += base.row(r) - base_anchor;
        traj.row(r) = hold + u * use;
    }
    return fit;
}

int execute_descent_table(
    Eigen::MatrixXd traj,
    int hand,
    int descent_row,
    Robot_Arm *arm,
    const Eigen::Matrix<double, 1, 6> &goal)
{
    if (hand == 1 && right_arm_motors_locked())
        return 0;
    if (hand < 0 || hand > 1 || arm == nullptr || traj.rows() < 2 || !goal.allFinite())
        return -10;
    if (descent_row < 1)
        descent_row = 1;
    if (descent_row >= traj.rows())
        descent_row = static_cast<int>(traj.rows()) - 1;

    {
        std::lock_guard<std::mutex> lock(g_desc[hand].mu);
        g_desc[hand].active = true;
        g_desc[hand].in_descent = false;
        g_desc[hand].ik_failed = false;
        g_desc[hand].cursor = 0;
        g_desc[hand].descent_row = descent_row;
        g_desc[hand].plan_rows = static_cast<int>(traj.rows());
        g_desc[hand].applied_x = goal(0);
        g_desc[hand].applied_y = goal(1);
        g_desc[hand].have_visual_xy = false;
        g_desc[hand].align_px = -1.0;
        g_desc[hand].goal = goal;
        g_desc[hand].arm = arm;
        g_desc[hand].traj = std::move(traj);
        g_desc[hand].base = g_desc[hand].traj;
    }
    struct Guard
    {
        int hand;
        ~Guard()
        {
            std::lock_guard<std::mutex> lock(g_desc[hand].mu);
            g_desc[hand].active = false;
            g_desc[hand].in_descent = false;
            g_desc[hand].arm = nullptr;
        }
    } guard{hand};

    const int dof_limit = 7;
    bool logged_start = false;
    Eigen::RowVectorXd last_sent;
    int i = 0;
    while (true)
    {
        if (hardware_abort_requested())
        {
            std::cout << "[arm] STOP，停止轨迹\n";
            return -4;
        }
        if (can_io_faulted())
        {
            std::cout << "[arm] CAN 收发失败，停止轨迹: " << can_io_fault_message() << std::endl;
            return -2;
        }

        Eigen::RowVectorXd row;
        int nrows = 0;
        {
            std::lock_guard<std::mutex> lock(g_desc[hand].mu);
            nrows = static_cast<int>(g_desc[hand].traj.rows());
            if (i < nrows)
            {
                row = g_desc[hand].traj.row(i);
                g_desc[hand].cursor = i;
                g_desc[hand].in_descent = i >= g_desc[hand].descent_row;
            }
        }
        if (i >= nrows)
            break;

        if (!logged_start && i >= descent_row)
        {
            logged_start = true;
            std::cout << "[descent] " << descent_hand_name(hand)
                      << " 下降开始。35% 修正一次，70% 再补一次，之后只监控\n";
        }

        const auto cycle_start = std::chrono::high_resolution_clock::now();
        const int dof = static_cast<int>(std::min(row.size(), static_cast<Eigen::Index>(dof_limit)));
        int motor_cmd[7] = {0};
        double q_rad[7] = {0, 0, 0, 0, 0, 0, 0};
        for (int j = 0; j < dof; ++j)
        {
            q_rad[j] = row(j);
            motor_cmd[j] = rad_to_encoder_cnt(row(j));
        }
        hand_shadow_note_q(hand, q_rad);
        set_motor_position(motor_cmd, hand);
        last_sent = row;

        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::high_resolution_clock::now() - cycle_start)
                .count();
        const double sleep_ms = std::max(0.0, k_arm_traj_dt * 1000.0 - elapsed_ms);
        if (sleep_ms > 1e-6)
            std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(sleep_ms));
        ++i;
    }

    if (last_sent.size() >= 1)
    {
        int last_cmd[7] = {0};
        const int dof = static_cast<int>(std::min(last_sent.size(), static_cast<Eigen::Index>(dof_limit)));
        for (int j = 0; j < dof; ++j)
            last_cmd[j] = rad_to_encoder_cnt(last_sent(j));
        save_arm_last_commanded_position(hand, last_cmd);
    }
    return 0;
}

} // namespace

bool descent_player_accepting(int hand)
{
    if (hand < 0 || hand > 1)
        return false;
    std::lock_guard<std::mutex> lock(g_desc[hand].mu);
    return g_desc[hand].active && g_desc[hand].in_descent && !g_desc[hand].ik_failed;
}

double descent_vertical_progress(int hand)
{
    if (hand < 0 || hand > 1)
        return -1.0;
    std::lock_guard<std::mutex> lock(g_desc[hand].mu);
    if (!g_desc[hand].active || !g_desc[hand].in_descent)
        return -1.0;
    const int span = g_desc[hand].plan_rows - g_desc[hand].descent_row;
    if (span <= 0)
        return 1.0;
    const double progress =
        static_cast<double>(g_desc[hand].cursor - g_desc[hand].descent_row) /
        static_cast<double>(span);
    if (progress < 0.0)
        return 0.0;
    if (progress > 1.0)
        return 1.0;
    return progress;
}

void descent_note_align_px(int hand, double px)
{
    if (hand < 0 || hand > 1)
        return;
    std::lock_guard<std::mutex> lock(g_desc[hand].mu);
    g_desc[hand].align_px = px;
}

void descent_set_visual_bias(int hand, double x_m, double y_m)
{
    if (hand < 0 || hand > 1)
        return;
    std::lock_guard<std::mutex> lock(g_desc[hand].mu);
    g_desc[hand].bias_x = std::isfinite(x_m) ? x_m : 0.0;
    g_desc[hand].bias_y = std::isfinite(y_m) ? y_m : 0.0;
}

void descent_visual_bias(int hand, double &x_m, double &y_m)
{
    x_m = 0.0;
    y_m = 0.0;
    if (hand < 0 || hand > 1)
        return;
    std::lock_guard<std::mutex> lock(g_desc[hand].mu);
    x_m = g_desc[hand].bias_x;
    y_m = g_desc[hand].bias_y;
}

bool descent_nudge_xy(int hand, double dx_m, double dy_m, const char *note)
{
    if (hand < 0 || hand > 1 || !std::isfinite(dx_m) || !std::isfinite(dy_m))
        return false;
    Robot_Arm *arm = nullptr;
    Eigen::RowVectorXd q_anchor;
    {
        std::lock_guard<std::mutex> lock(g_desc[hand].mu);
        if (!g_desc[hand].active || !g_desc[hand].in_descent || g_desc[hand].arm == nullptr)
            return false;
        if (g_desc[hand].traj.rows() < 2)
            return false;
        int anchor = g_desc[hand].cursor;
        if (anchor < 0)
            anchor = 0;
        if (anchor >= g_desc[hand].traj.rows())
            anchor = static_cast<int>(g_desc[hand].traj.rows()) - 1;
        arm = g_desc[hand].arm;
        q_anchor = g_desc[hand].traj.row(anchor);
    }
    double x = 0.0;
    double y = 0.0;
    if (arm == nullptr || !tcp_xy_of(*arm, q_anchor, x, y))
        return false;
    return descent_apply_hole_xy(hand, x + dx_m, y + dy_m, note);
}

bool descent_commanded_xy(int hand, double &x_m, double &y_m)
{
    x_m = 0.0;
    y_m = 0.0;
    if (hand < 0 || hand > 1)
        return false;
    std::lock_guard<std::mutex> lock(g_desc[hand].mu);
    if (!g_desc[hand].have_visual_xy)
        return false;
    x_m = g_desc[hand].applied_x;
    y_m = g_desc[hand].applied_y;
    return std::isfinite(x_m) && std::isfinite(y_m);
}

bool descent_apply_hole_xy(int hand, double base_x, double base_y, const char *note)
{
    if (hand < 0 || hand > 1 || !std::isfinite(base_x) || !std::isfinite(base_y))
        return false;

    Robot_Arm *arm = nullptr;
    Eigen::RowVectorXd q_anchor;
    Eigen::RowVectorXd q_hold_last;
    int anchor = 0;
    {
        std::lock_guard<std::mutex> lock(g_desc[hand].mu);
        DescentSlot &slot = g_desc[hand];
        if (!slot.active || !slot.in_descent || slot.arm == nullptr)
            return false;
        if (slot.traj.rows() < 2 || slot.traj.cols() < 7 || slot.base.rows() != slot.traj.rows() ||
            slot.base.cols() != slot.traj.cols())
            return false;
        anchor = slot.cursor;
        if (anchor < 0)
            anchor = 0;
        if (anchor >= slot.traj.rows() - 1)
            return true;
        arm = slot.arm;
        q_anchor = slot.traj.row(anchor);
        q_hold_last = q_anchor + (slot.base.row(slot.base.rows() - 1) - slot.base.row(anchor));
    }

    double cx = 0.0;
    double cy = 0.0;
    if (arm == nullptr || !tcp_xy_of(*arm, q_anchor, cx, cy))
        return false;
    const double dx = base_x - cx;
    const double dy = base_y - cy;

    Eigen::RowVectorXd dq = Eigen::RowVectorXd::Zero(q_anchor.size());
    if (std::hypot(dx, dy) >= 0.0005)
    {
        if (!tcp_xy_joint_step(*arm, q_anchor, q_hold_last, dx, dy, dq))
        {
            std::cout << "[descent] " << descent_hand_name(hand)
                      << " 这一帧水平增量没算出来，下降按原表继续\n";
            return false;
        }
    }

    {
        std::lock_guard<std::mutex> lock(g_desc[hand].mu);
        DescentSlot &slot = g_desc[hand];
        if (!slot.active || slot.traj.rows() < 2 || slot.base.rows() != slot.traj.rows() ||
            slot.traj.cols() != dq.size())
            return false;
        anchor = slot.cursor;
        if (anchor < 0)
            anchor = 0;
        if (anchor >= slot.traj.rows() - 1)
            return true;
        int from = anchor + 1;
        if (from < slot.descent_row)
            from = slot.descent_row;
        if (from >= slot.traj.rows())
            return true;
        const Eigen::RowVectorXd hold_last =
            slot.traj.row(anchor) +
            (slot.base.row(slot.base.rows() - 1) - slot.base.row(anchor));
        const double pending = (slot.traj.row(slot.traj.rows() - 1) - hold_last).cwiseAbs().maxCoeff();
        if (std::hypot(dx, dy) < 0.0005 && pending < 1e-3)
            return true;
        const double fit = rewrite_unsent_xy(slot.traj, slot.base, anchor, from, dq);
        slot.applied_x = cx + dx * fit;
        slot.applied_y = cy + dy * fit;
        slot.have_visual_xy = true;
        std::cout << std::fixed << std::setprecision(1)
                  << "[descent] " << descent_hand_name(hand)
                  << " 水平 " << (std::hypot(dx, dy) * fit * 1000.0)
                  << " mm，按这一帧重写未下发下降，从第 " << from << " 行起"
                  << (note != nullptr && note[0] != '\0' ? "  " : "")
                  << (note != nullptr ? note : "") << '\n';
    }
    return true;
}

ArmLineMoveResult arm_dual_line_then_line_nostop(
    Robot_Arm &arm_r,
    const Eigen::Matrix<double, 1, 6> &mid_r,
    const Eigen::Matrix<double, 1, 6> &goal_r,
    const bool move_r,
    Robot_Arm &arm_l,
    const Eigen::Matrix<double, 1, 6> &mid_l,
    const Eigen::Matrix<double, 1, 6> &goal_l,
    const bool move_l,
    const double vel_first_m_s,
    const double vel_second_m_s,
    const std::function<void()> &before_move,
    const bool use_prepared,
    const char *log_tag,
    const bool snap_reach,
    const bool correct_descent,
    const int hand_target_class)
{
    ArmLineMoveResult out;
    Eigen::MatrixXd traj_r, traj_l;
    const bool prepared_r =
        use_prepared && !correct_descent && move_r &&
        take_prepared_line(arm_hand_side(arm_r), traj_r, out.ret_r);
    const bool prepared_l =
        use_prepared && !correct_descent && move_l &&
        take_prepared_line(arm_hand_side(arm_l), traj_l, out.ret_l);
    if (move_r && !prepared_r)
        out.ret_r = plan_line_then_line(
            arm_r, mid_r, goal_r, vel_first_m_s, vel_second_m_s, traj_r, log_tag);
    if (move_l && !prepared_l)
        out.ret_l = plan_line_then_line(
            arm_l, mid_l, goal_l, vel_first_m_s, vel_second_m_s, traj_l, log_tag);
    if ((move_r && out.ret_r != 0) || (move_l && out.ret_l != 0))
    {
        if (move_r && out.ret_r == 0)
            out.ret_r = -14;
        if (move_l && out.ret_l == 0)
            out.ret_l = -14;
        return out;
    }
    if (before_move)
        before_move();
    if (correct_descent)
        g_descent_fail_hand.store(-1, std::memory_order_release);
    const auto watch = [&](Robot_Arm &arm, const Eigen::Matrix<double, 1, 6> &goal) {
        const int hand = arm_hand_side(arm);
        if (correct_descent)
            hand_shadow_ensure(
                hand, &arm, goal, log_tag != nullptr ? log_tag : "下降", hand_target_class);
    };
    if (move_r)
        watch(arm_r, goal_r);
    if (move_l)
        watch(arm_l, goal_l);
    auto run_one = [&](Robot_Arm &arm, const Eigen::MatrixXd &traj,
                       const Eigen::Matrix<double, 1, 6> &goal, int &ret) {
        const int hand = arm_hand_side(arm);
        if (correct_descent)
            ret = execute_descent_table(traj, hand, g_planned_descent_row[hand], &arm, goal);
        else
            ret = execute_arm_trajectory(traj, hand, k_arm_traj_dt);
    };
    if (move_r && move_l)
    {
        std::thread th_r([&]() { run_one(arm_r, traj_r, goal_r, out.ret_r); });
        std::thread th_l([&]() { run_one(arm_l, traj_l, goal_l, out.ret_l); });
        th_r.join();
        th_l.join();
    }
    else if (move_r)
        run_one(arm_r, traj_r, goal_r, out.ret_r);
    else if (move_l)
        run_one(arm_l, traj_l, goal_l, out.ret_l);
    if (correct_descent && move_r)
        hand_shadow_end(arm_hand_side(arm_r));
    if (correct_descent && move_l)
        hand_shadow_end(arm_hand_side(arm_l));
    const char *done = correct_descent ? "下降到位" : (snap_reach ? "伸手到位" : "收手终点");
    if (move_r && out.ret_r == 0)
        record_arm_cartesian_arrival(arm_r, goal_r, done);
    if (move_l && out.ret_l == 0)
        record_arm_cartesian_arrival(arm_l, goal_l, done);
    return out;
}

ArmLineMoveResult arm_dual_rounded_corner_move_selective(
    Robot_Arm &arm_r,
    const Matrix<double, 1, 6> &hover_r,
    const Matrix<double, 1, 6> &final_r,
    const bool move_r,
    Robot_Arm &arm_l,
    const Matrix<double, 1, 6> &hover_l,
    const Matrix<double, 1, 6> &final_l,
    const bool move_l,
    const double cruise_mps)
{
    ArmLineMoveResult out;
    Eigen::MatrixXd traj_r, traj_l;
    if (move_r)
        out.ret_r = plan_rounded_corner(arm_r, hover_r, final_r, cruise_mps, traj_r);
    if (move_l)
        out.ret_l = plan_rounded_corner(arm_l, hover_l, final_l, cruise_mps, traj_l);
    if ((move_r && out.ret_r != 0) || (move_l && out.ret_l != 0))
    {
        if (move_r && out.ret_r == 0)
            out.ret_r = -14;
        if (move_l && out.ret_l == 0)
            out.ret_l = -14;
        std::cerr << "[corner] 抹角规划失败，不回退分段直线。右=" << out.ret_r
                  << " 左=" << out.ret_l << '\n';
        return out;
    }

    auto run_one = [&](Robot_Arm &arm, const Eigen::MatrixXd &traj, int &ret) {
        const int hand = arm_hand_side(arm);
        ret = execute_arm_trajectory(traj, hand, k_arm_traj_dt);
    };
    if (move_r && move_l)
    {
        std::thread th_r([&]() { run_one(arm_r, traj_r, out.ret_r); });
        std::thread th_l([&]() { run_one(arm_l, traj_l, out.ret_l); });
        th_r.join();
        th_l.join();
    }
    else if (move_r)
        run_one(arm_r, traj_r, out.ret_r);
    else if (move_l)
        run_one(arm_l, traj_l, out.ret_l);
    if (move_r && out.ret_r == 0)
        record_arm_cartesian_arrival(arm_r, final_r, "抹角C");
    if (move_l && out.ret_l == 0)
        record_arm_cartesian_arrival(arm_l, final_l, "抹角C");
    return out;
}


int arm_transfer_move(Robot_Arm &arm, Matrix<double, 1, 6> &pos, double cart_linear_velocity)
{
    return arm_line_move(arm, pos, cart_linear_velocity);
}

ArmLineMoveResult arm_dual_transfer_move(
    Robot_Arm &arm_r,
    Matrix<double, 1, 6> &pos_r,
    Robot_Arm &arm_l,
    Matrix<double, 1, 6> &pos_l,
    double cart_linear_velocity)
{
    ArmLineMoveResult out;
    std::thread th_r([&]() { out.ret_r = arm_transfer_move(arm_r, pos_r, cart_linear_velocity); });
    std::thread th_l([&]() { out.ret_l = arm_transfer_move(arm_l, pos_l, cart_linear_velocity); });
    th_r.join();
    th_l.join();
    return out;
}

ArmLineMoveResult arm_dual_transfer_move_selective(
    Robot_Arm &arm_r,
    Matrix<double, 1, 6> &pos_r,
    const bool move_r,
    Robot_Arm &arm_l,
    Matrix<double, 1, 6> &pos_l,
    const bool move_l,
    const double cart_linear_velocity)
{
    if (move_r && move_l)
        return arm_dual_transfer_move(arm_r, pos_r, arm_l, pos_l, cart_linear_velocity);

    ArmLineMoveResult out;
    if (move_r)
        out.ret_r = arm_transfer_move(arm_r, pos_r, cart_linear_velocity);
    if (move_l)
        out.ret_l = arm_transfer_move(arm_l, pos_l, cart_linear_velocity);
    return out;
}

namespace
{

constexpr int kMotorStoppedMinZeroCount = 6;
constexpr int kEncoderErrStableDelta = 10;    // plateau 判定：相邻两次读数差值上限
constexpr int kEncoderStablePlateauPolls = 10; // 误差稳定连续次数
constexpr int kEncoderPlateauMaxAbsErr = 2500; // 约 3.4°；急停后误差很大且不动，不能当到位
constexpr int kEncoderPollIntervalMs = 50;   // 停稳轮询/读编码器间隔

struct ArmEncoderReachState
{
    int prev_err[7] = {0};
    bool have_prev = false;
    int plateau_count = 0;
};

struct ArmEncoderReachResult
{
    bool ok = false;
    bool used_plateau = false;
};

ArmEncoderReachResult check_arm_encoder_reach(int hand, ArmEncoderReachState &st)
{
    ArmEncoderReachResult res;
    if (hand < 0 || hand > 1 || !g_arm_last_cmd_valid[hand])
    {
        res.ok = true;
        return res;
    }

    int actual[7] = {0};
    get_motor_position(actual, hand);
    int err[7] = {0};
    for (int i = 0; i < 7; ++i)
        err[i] = actual[i] - g_arm_last_cmd_cnt[hand][i];

    bool all_strict = true;
    for (int i = 0; i < 7; ++i)
    {
        if (std::abs(err[i]) > kMotorPositionReachTolerance)
            all_strict = false;
    }

    if (all_strict)
    {
        st.plateau_count = 0;
        st.have_prev = false;
        res.ok = true;
        return res;
    }

    // |err|>550 且连续稳定：仅当误差仍在合理带内才视为到位（急停/失能使能后误差很大且不动）。
    bool err_too_large = false;
    for (int i = 0; i < 7; ++i)
    {
        if (std::abs(err[i]) > kEncoderPlateauMaxAbsErr)
            err_too_large = true;
    }
    if (st.have_prev && !err_too_large)
    {
        bool stable = true;
        for (int i = 0; i < 7; ++i)
        {
            if (std::abs(err[i]) > kMotorPositionReachTolerance)
            {
                if (std::abs(err[i] - st.prev_err[i]) > kEncoderErrStableDelta)
                    stable = false;
            }
        }
        if (stable)
            ++st.plateau_count;
        else
            st.plateau_count = 0;
    }
    else
    {
        st.plateau_count = 0;
    }

    for (int i = 0; i < 7; ++i)
        st.prev_err[i] = err[i];
    st.have_prev = true;

    if (st.plateau_count >= kEncoderStablePlateauPolls)
    {
        res.ok = true;
        res.used_plateau = true;
    }
    return res;
}

bool arms_encoder_reach_ok(
    bool wait_r,
    bool wait_l,
    ArmEncoderReachState &st_r,
    ArmEncoderReachState &st_l,
    bool &used_plateau)
{
    used_plateau = false;
    if (wait_r)
    {
        const auto r = check_arm_encoder_reach(1, st_r);
        if (!r.ok)
            return false;
        if (r.used_plateau)
            used_plateau = true;
    }
    if (wait_l)
    {
        const auto l = check_arm_encoder_reach(0, st_l);
        if (!l.ok)
            return false;
        if (l.used_plateau)
            used_plateau = true;
    }
    return true;
}

void reset_arm_encoder_reach_state(ArmEncoderReachState &st)
{
    st = {};
}

bool motor_joint_speed_counts_stopped(int speed)
{
    return speed == 0;
}

void log_motor_running_speeds(const char *label, const int speed[7])
{
    std::cout << label;
    for (int i = 0; i < 7; ++i)
        std::cout << ' ' << speed[i];
    std::cout << std::endl << std::flush;
}

} // namespace

void log_phase_banner(const char *title)
{
    const char *t = (title != nullptr && title[0] != '\0') ? title : "(未命名阶段)";
    std::cout << kLogPhaseSep << '\n'
              << ">>> " << t << " <<<\n"
              << kLogPhaseSep << std::endl;
}

void log_cycle_banner(int round_index)
{
    std::cout << kLogCycleSep << '\n';
    if (round_index >= 0)
        std::cout << "**** 搬箱主循环 第 " << round_index << " 轮 ****\n";
    else
        std::cout << "**** 搬箱主循环 新一轮 ****\n";
    std::cout << kLogCycleSep << std::endl;
}

void log_arms_motor_running_speeds(bool log_r, bool log_l, const char *reason)
{
    int speed_r[7] = {0};
    int speed_l[7] = {0};
    if (log_r)
        get_motor_running_speed(speed_r, 1);
    if (log_l)
        get_motor_running_speed(speed_l, 0);
    std::cout << "[arm] " << reason;
    if (log_r)
    {
        int zero_r = 0;
        std::cout << " speed_r=";
        for (int i = 0; i < 7; ++i)
        {
            std::cout << ' ' << speed_r[i];
            if (speed_r[i] == 0)
                ++zero_r;
        }
        std::cout << " (zero=" << zero_r << "/7)";
    }
    if (log_l)
    {
        int zero_l = 0;
        std::cout << " speed_l=";
        for (int i = 0; i < 7; ++i)
        {
            std::cout << ' ' << speed_l[i];
            if (speed_l[i] == 0)
                ++zero_l;
        }
        std::cout << " (zero=" << zero_l << "/7)";
    }
    std::cout << std::endl << std::flush;
}

bool arm_motors_all_stopped(int hand, int speed_threshold)
{
    (void)speed_threshold;
    int speed[7] = {0};
    get_motor_running_speed(speed, hand);

    int zero_count = 0;
    for (int i = 0; i < 7; ++i)
    {
        if (motor_joint_speed_counts_stopped(speed[i]))
            ++zero_count;
    }
    return zero_count >= kMotorStoppedMinZeroCount;
}

bool wait_arms_motors_stopped(
    bool wait_r,
    bool wait_l,
    double timeout_sec,
    int min_zero_count,
    const char *log_stage)
{
    if (right_arm_motors_locked())
        wait_r = false;
    if (!wait_r && !wait_l)
        return true;

    const char *stage = (log_stage != nullptr && log_stage[0] != '\0') ? log_stage : "手相机前";

#if 0 // 速度停稳检查暂不用，保留便于恢复
    const int required_zero =
        (min_zero_count > 0 && min_zero_count <= 7) ? min_zero_count : kMotorStoppedMinZeroCount;

    auto side_stopped = [required_zero](int hand) -> bool {
        int speed[7] = {0};
        get_motor_running_speed(speed, hand);
        int zero_count = 0;
        for (int i = 0; i < 7; ++i)
        {
            if (motor_joint_speed_counts_stopped(speed[i]))
                ++zero_count;
        }
        return zero_count >= required_zero;
    };

    constexpr int kRequiredStablePolls = 6; // 连续 6 次速度停稳（约 300ms）
    int stable_count = 0;
#endif
    (void)min_zero_count;

    ArmEncoderReachState enc_r;
    ArmEncoderReachState enc_l;

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(timeout_sec));
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (hardware_abort_requested())
        {
            std::cerr << "[arm] STOP：停稳等待中止\n";
            return false;
        }
        if (can_io_faulted())
        {
            std::cerr << "[arm] CAN 失败：停稳等待中止\n";
            return false;
        }
#if 0 // 速度停稳检查暂不用，保留便于恢复
        bool ok = true;
        if (wait_r)
            ok = ok && side_stopped(1);
        if (wait_l)
            ok = ok && side_stopped(0);
        if (ok)
        {
            ++stable_count;
            if (stable_count >= kRequiredStablePolls)
            {
#endif
                bool used_plateau = false;
                if (arms_encoder_reach_ok(wait_r, wait_l, enc_r, enc_l, used_plateau))
                {
                    const std::string ok_msg = used_plateau
                        ? std::string(stage) + "停稳OK(plateau>±550)"
                        : std::string(stage) + "停稳OK(±550)";
                    log_arms_motor_position_reach(wait_r, wait_l, ok_msg.c_str());
                    return true;
                }
#if 0 // 速度停稳检查暂不用，保留便于恢复
            }
        }
        else
        {
            stable_count = 0;
            reset_arm_encoder_reach_state(enc_r);
            reset_arm_encoder_reach_state(enc_l);
        }
#endif
        std::this_thread::sleep_for(std::chrono::milliseconds(kEncoderPollIntervalMs));
    }

    const std::string timeout_msg =
        std::string(stage) + "停稳失败(超时, 编码器±550/plateau>±550)";
    log_arms_motor_position_reach(wait_r, wait_l, timeout_msg.c_str());
    std::cerr << "[arm] 等待电机停稳超时 wait_r=" << wait_r << " wait_l=" << wait_l << std::endl;
    return false;
}

namespace
{

std::mutex g_arm_arrival_mu;
std::vector<ArmArrivalRecord> g_arm_arrivals;
constexpr std::size_t kArmArrivalLogMax = 2000;

} // namespace

void arm_arrival_log_clear()
{
    std::lock_guard<std::mutex> lock(g_arm_arrival_mu);
    g_arm_arrivals.clear();
}

std::vector<ArmArrivalRecord> arm_arrival_log_copy()
{
    std::lock_guard<std::mutex> lock(g_arm_arrival_mu);
    return g_arm_arrivals;
}

bool record_arm_cartesian_arrival(
    Robot_Arm &arm,
    const Eigen::Matrix<double, 1, 6> &goal,
    const char *stage,
    double settle_sec)
{
    const int hand = arm_hand_side(arm);
    if (hand == 1 && right_arm_motors_locked())
        return false;

    std::string stage_txt;
    if (stage != nullptr && stage[0] != '\0')
        stage_txt = stage;
    else
    {
        std::lock_guard<std::mutex> lock(g_arm_line_move_stage_mu);
        stage_txt = (g_arm_line_move_debug_stage != nullptr && g_arm_line_move_debug_stage[0] != '\0')
                        ? g_arm_line_move_debug_stage
                        : "直线";
    }

    const bool is_right = hand == 1;
    const bool settled = wait_arms_motors_stopped(
        is_right, !is_right, settle_sec, 6, stage_txt.c_str());
    const Eigen::Matrix<double, 1, 6> actual = arm_get_tcp_pos(arm);

    ArmArrivalRecord rec;
    rec.stage = stage_txt;
    rec.hand = is_right ? "右" : "左";
    rec.goal_x = goal(0);
    rec.goal_y = goal(1);
    rec.goal_z = goal(2);
    rec.actual_x = actual(0);
    rec.actual_y = actual(1);
    rec.actual_z = actual(2);
    rec.err_x = actual(0) - goal(0);
    rec.err_y = actual(1) - goal(1);
    rec.err_z = actual(2) - goal(2);
    rec.err_m = std::sqrt(rec.err_x * rec.err_x + rec.err_y * rec.err_y + rec.err_z * rec.err_z);
    rec.settled = settled;

    {
        std::lock_guard<std::mutex> lock(g_arm_arrival_mu);
        if (g_arm_arrivals.size() < kArmArrivalLogMax)
            g_arm_arrivals.push_back(rec);
    }

    std::cout << std::fixed << std::setprecision(4)
              << "[arm] 到位 " << rec.hand << " " << rec.stage
              << " 目标xyz=(" << rec.goal_x << "," << rec.goal_y << "," << rec.goal_z
              << ") 实际xyz=(" << rec.actual_x << "," << rec.actual_y << "," << rec.actual_z
              << ") 误差xyz=(" << rec.err_x << "," << rec.err_y << "," << rec.err_z
              << ") |err|=" << rec.err_m << "m"
              << (settled ? " 停稳" : " 未停稳") << "\n";
    std::cout.flush();
    return settled;
}

int arm_joint_move(Robot_Arm &arm, Matrix<double, 1, 7> &qd)
{
    const int hand = arm_hand_side(arm);
    if (hand == 1 && right_arm_motors_locked())
        return 0;
    const Eigen::MatrixXd q_start = arm_read_motor_joints(hand);
    Eigen::MatrixXd q_end = qd;
    if (q_end.rows() != 1)
        q_end = q_end.transpose();
    Eigen::MatrixXd traj;
    const int ret = arm.Joint_Trajectory(q_start, q_end, traj);
    if (ret != 0)
    {
        std::cerr << "[arm] Joint_Trajectory ret=" << ret
                  << "（起点可在模型限位外），改用不限位关节插补回目标\n";
        traj = make_linear_joint_traj(q_start, q_end, k_arm_traj_dt);
    }
    return execute_arm_trajectory(traj, hand, k_arm_traj_dt);
}

Eigen::Matrix4d arm_fk_tcp_from_encoders(Robot_Arm &arm)
{
    const int hand = arm_hand_side(arm);
    const Eigen::MatrixXd q = arm_read_motor_joints(hand);
    return arm.Forward_Kinematics(q.row(0));
}

Matrix<double, 1, 6> arm_get_tcp_pos(Robot_Arm &arm)
{
    return T2PosEulerAngles(arm_fk_tcp_from_encoders(arm));
}

Matrix<double, 1, 6> compose_base_visual(
    Robot_Arm &arm,
    const Eigen::Matrix4d &T_tcp,
    Matrix<double, 1, 6> &falan_visualPos)
{
    const Matrix<double, 1, 6> flange_tool = arm_flange_tool(arm);
    const Eigen::Matrix4d T_base_visual = T_tcp * TR(flange_tool).inverse() * TR(falan_visualPos);
    return T2PosEulerAngles(T_base_visual);
}

Matrix<double, 1, 6> arm_get_base_visual_pos(Robot_Arm &arm, Matrix<double, 1, 6> &falan_visualPos)
{
    return compose_base_visual(arm, arm_fk_tcp_from_encoders(arm), falan_visualPos);
}

Matrix<double, 1, 6> arm_base_visual_from_joints(
    Robot_Arm &arm,
    const Eigen::Ref<const Eigen::RowVectorXd> &q_motor,
    Matrix<double, 1, 6> &falan_visualPos)
{
    std::lock_guard<std::mutex> kin(g_arm_kin_mu[arm_hand_side(arm) == 1 ? 1 : 0]);
    return compose_base_visual(arm, arm.Forward_Kinematics(q_motor), falan_visualPos);
}

Matrix<double, 1, 6> arm_tcp_from_joints(
    Robot_Arm &arm,
    const Eigen::Ref<const Eigen::RowVectorXd> &q_motor)
{
    std::lock_guard<std::mutex> kin(g_arm_kin_mu[arm_hand_side(arm) == 1 ? 1 : 0]);
    return T2PosEulerAngles(arm.Forward_Kinematics(q_motor));
}

bool arm_camera_axes_in_base(
    Robot_Arm &arm,
    const Eigen::Ref<const Eigen::RowVectorXd> &q_motor,
    const std::array<double, 16> &t_flange_cam,
    Eigen::Vector3d &origin_m,
    Eigen::Vector3d &axis_x,
    Eigen::Vector3d &axis_y,
    Eigen::Vector3d &axis_z)
{
    if (q_motor.size() < 7)
        return false;
    std::lock_guard<std::mutex> kin(g_arm_kin_mu[arm_hand_side(arm) == 1 ? 1 : 0]);
    const Eigen::Matrix4d T_tcp = arm.Forward_Kinematics(q_motor);
    const Eigen::Matrix4d T_flange = T_tcp * TR(arm_flange_tool(arm)).inverse();
    Eigen::Matrix4d T_fc = Eigen::Matrix4d::Identity();
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            T_fc(r, c) = t_flange_cam[static_cast<std::size_t>(r * 4 + c)];
    const Eigen::Matrix4d T_cam = T_flange * T_fc;
    if (!T_cam.allFinite())
        return false;
    origin_m = T_cam.block<3, 1>(0, 3);
    axis_x = T_cam.block<3, 1>(0, 0);
    axis_y = T_cam.block<3, 1>(0, 1);
    axis_z = T_cam.block<3, 1>(0, 2);
    if (axis_x.norm() < 1e-9 || axis_y.norm() < 1e-9 || axis_z.norm() < 1e-9)
        return false;
    axis_x.normalize();
    axis_y.normalize();
    axis_z.normalize();
    return true;
}

int arm_wave_right(Robot_Arm &arm_r, Robot_Arm &arm_l)
{
    if (arm_hand_side(arm_r) != 1 || arm_hand_side(arm_l) != 0)
    {
        std::cerr << "[wave] 需要右臂和左臂\n";
        return -1;
    }
    if (right_arm_motors_locked())
    {
        std::cerr << "[wave] 右臂电机锁定，不挥手\n";
        return -6;
    }
    if (hardware_abort_requested())
        return -4;

    const WaveQ q_hang_l = -k_wave_hang;
    const Eigen::MatrixXd sine = wave_sine_traj();
    if (wave_log_over_limit(arm_r, 1, k_wave_hang, " 下垂") != 0 ||
        wave_log_over_limit(arm_r, 1, k_wave_front, " 身前") != 0 ||
        wave_log_over_limit(arm_r, 1, k_wave_raise, " 举起") != 0 ||
        wave_log_over_limit(arm_r, 1, k_wave_out, " 靠右") != 0 ||
        wave_log_over_limit(arm_r, 1, k_wave_in, " 收回") != 0 ||
        wave_log_over_limit(arm_l, 0, q_hang_l, " 下垂") != 0 ||
        wave_check_traj(arm_r, 1, sine, " 挥动") != 0)
        return -1;

    int rc = wave_head_zero();
    if (rc != 0)
        return rc;
    std::cout << "[wave] 左手收到下垂\n";
    rc = wave_goto(arm_l, q_hang_l, -1.0);
    if (rc != 0)
        return rc;
    std::cout << "[wave] 右手收到下垂\n";
    rc = wave_goto(arm_r, k_wave_hang, -1.0);
    if (rc != 0)
        return rc;
    orch::wave_grip_ratio(false, 1.0);
    orch::wave_grip_ratio(true, 1.0);
    if (hardware_abort_requested())
        return -4;
    std::cout << "[wave] 右手经身前抬到肩高\n";
    WaveQ q_lift = k_wave_hang;
    const Eigen::MatrixXd q_at_hang = arm_read_motor_joints(1);
    if (can_io_faulted() || !q_at_hang.allFinite())
        return -2;
    for (int j = 0; j < 7; ++j)
        q_lift(j) = q_at_hang(0, j);
    const Eigen::MatrixXd lift = wave_via(q_lift, k_wave_front, k_wave_raise, 1.0, 0.8);
    if (wave_check_traj(arm_r, 1, lift, " 抬手") != 0)
        return -1;
    rc = wave_play(1, lift, false);
    if (rc != 0)
        return rc;
    rc = wave_pause_ms(250);
    if (rc != 0)
        return rc;
    orch::wave_grip_ratio(true, k_wave_grip_ready);
    if (hardware_abort_requested())
        return -4;
    std::cout << "[wave] 右手挥 3 下，头轻微点头\n";
    rc = wave_play(1, sine, true);
    if (rc != 0)
    {
        orch::wave_grip_ratio(true, 1.0);
        return rc;
    }
    std::cout << "[wave] 右手连续放下\n";
    const Eigen::MatrixXd lower = wave_via(k_wave_raise, k_wave_front, k_wave_hang, 0.9, 1.1);
    if (wave_check_traj(arm_r, 1, lower, " 放下") != 0)
    {
        orch::wave_grip_ratio(true, 1.0);
        return -1;
    }
    rc = wave_play(1, lower, false);
    orch::wave_grip_ratio(true, 1.0);
    if (rc != 0)
        return rc;
    std::cout << "[wave] 完成，双手停在下垂\n";
    return 0;
}

int arm_wave_hang(Robot_Arm &arm_r, Robot_Arm &arm_l)
{
    if (arm_hand_side(arm_r) != 1 || arm_hand_side(arm_l) != 0)
    {
        std::cerr << "[wave] 需要右臂和左臂\n";
        return -1;
    }
    if (right_arm_motors_locked())
    {
        std::cerr << "[wave] 右臂电机锁定，不进下垂准备\n";
        return -6;
    }
    if (hardware_abort_requested())
        return -4;

    const WaveQ q_hang_l = -k_wave_hang;
    if (wave_log_over_limit(arm_r, 1, k_wave_hang, " 下垂") != 0 ||
        wave_log_over_limit(arm_l, 0, q_hang_l, " 下垂") != 0)
        return -1;

    std::cout << "[wave] 左手收到下垂\n";
    int rc = wave_goto(arm_l, q_hang_l, -1.0);
    if (rc != 0)
        return rc;
    std::cout << "[wave] 右手收到下垂\n";
    rc = wave_goto(arm_r, k_wave_hang, -1.0);
    if (rc != 0)
        return rc;
    orch::wave_grip_ratio(false, 1.0);
    orch::wave_grip_ratio(true, 1.0);
    std::cout << "[wave] 双手已在下垂\n";
    return hardware_abort_requested() ? -4 : 0;
}

int arm_bow(Robot_Arm &arm_r, Robot_Arm &arm_l)
{
    if (arm_hand_side(arm_r) != 1 || arm_hand_side(arm_l) != 0)
    {
        std::cerr << "[bow] 需要右臂和左臂\n";
        return -1;
    }
    if (right_arm_motors_locked())
    {
        std::cerr << "[bow] 右臂电机锁定，不鞠躬\n";
        return -6;
    }
    if (hardware_abort_requested())
        return -4;

    constexpr double k_bow_slow = 2.0;
    constexpr double k_deg = M_PI / 180.0;
    const WaveQ q_hang_l = -k_wave_hang;
    const WaveQ q_chest = wave_q(0.3856, 1.0966, -0.8351, 1.9461, -0.4402, 0.6800, 0.1030);
    const WaveQ q_via = wave_q(0.18, 1.22, -0.70, 0.95, -0.50, 0.48, 0.10);
    WaveQ q_bow_l = q_hang_l;
    q_bow_l(3) -= 12.0 * k_deg;
    const Eigen::MatrixXd lift = bow_rest_via(
        k_wave_hang, q_via, q_chest, 1.0 * k_bow_slow, 0.8 * k_bow_slow);
    const Eigen::MatrixXd lower = bow_rest_via(
        q_chest, q_via, k_wave_hang, 0.9 * k_bow_slow, 1.1 * k_bow_slow);
    if (wave_log_over_limit(arm_r, 1, k_wave_hang, " 下垂") != 0 ||
        wave_log_over_limit(arm_r, 1, q_via, " 身前") != 0 ||
        wave_log_over_limit(arm_r, 1, q_chest, " 胸前") != 0 ||
        wave_log_over_limit(arm_l, 0, q_hang_l, " 下垂") != 0 ||
        wave_log_over_limit(arm_l, 0, q_bow_l, " 随腰弯肘") != 0 ||
        wave_check_traj(arm_r, 1, lift, " 收到胸前") != 0 ||
        wave_check_traj(arm_r, 1, lower, " 放回下垂") != 0)
        return -1;

    int rc = wave_head_zero();
    if (rc != 0)
        return rc;
    std::cout << "[bow] 双手进入下垂，速度按 " << k_bow_slow << " 倍放慢\n";
    rc = wave_goto(arm_l, q_hang_l, -1.0);
    if (rc != 0)
        return rc;
    rc = wave_goto(arm_r, k_wave_hang, -1.0);
    if (rc != 0)
        return rc;
    orch::wave_grip_ratio(false, 1.0);
    orch::wave_grip_ratio(true, 0.2);
    rc = wave_pause_ms(static_cast<int>(std::lround(200.0 * k_bow_slow)));
    if (rc != 0)
        return rc;

    std::cout << "[bow] 右手经身前收到胸前\n";
    rc = wave_play(1, lift, false);
    if (rc != 0)
        return rc;
    rc = wave_pause_ms(static_cast<int>(std::lround(250.0 * k_bow_slow)));
    if (rc != 0)
        return rc;

    const BowWaist w0 = bow_read_waist();
    BowWaist w1 = w0;
    w1.ankle += 5.0 * k_deg;
    w1.knee += -3.0 * k_deg;
    w1.hip += -27.0 * k_deg;
    w1.roll = 0.0;
    w1.yaw = 0.0;
    const double pitch0 = (-w0.ankle - w0.knee - w0.hip) / k_deg;
    const double pitch1 = (-w1.ankle - w1.knee - w1.hip) / k_deg;
    std::cout << std::fixed << std::setprecision(1)
              << "[bow] 腰前倾 " << (pitch1 - pitch0)
              << "°（现 " << pitch0 << "° → " << pitch1
              << "°），下去 " << (1.2 * k_bow_slow) << "s，停 "
              << (0.7 * k_bow_slow) << "s，起来 " << (1.4 * k_bow_slow) << "s\n";
    constexpr double k_head0 = 10.0;
    constexpr double k_head1 = 18.0;
    rc = bow_play(q_chest, q_chest, q_hang_l, q_bow_l, w0, w1, k_head0, k_head1,
                  1.2 * k_bow_slow);
    if (rc != 0)
        return rc;
    rc = bow_play(q_chest, q_chest, q_bow_l, q_bow_l, w1, w1, k_head1, k_head1,
                  0.7 * k_bow_slow);
    if (rc != 0)
        return rc;
    rc = bow_play(q_chest, q_chest, q_bow_l, q_hang_l, w1, w0, k_head1, k_head0,
                  1.4 * k_bow_slow);
    if (rc != 0)
        return rc;

    std::cout << "[bow] 右手放回下垂\n";
    rc = wave_play(1, lower, false);
    if (rc != 0)
        return rc;
    orch::wave_grip_ratio(true, 1.0);
    std::cout << "[bow] 完成，双手停在下垂\n";
    return hardware_abort_requested() ? -4 : 0;
}

int arm_welcome(Robot_Arm &arm_r, Robot_Arm &arm_l)
{
    if (arm_hand_side(arm_r) != 1 || arm_hand_side(arm_l) != 0)
    {
        std::cerr << "[welcome] 需要右臂和左臂\n";
        return -1;
    }
    if (right_arm_motors_locked())
    {
        std::cerr << "[welcome] 右臂电机锁定，不迎宾\n";
        return -6;
    }
    if (hardware_abort_requested())
        return -4;

    const WaveQ q_hang_l = -k_wave_hang;
    int rc = wave_head_zero();
    if (rc != 0)
        return rc;
    std::cout << "[welcome] 双手进入下垂准备，夹爪闭合\n";
    rc = wave_goto(arm_l, q_hang_l, -1.0);
    if (rc != 0)
        return rc;
    rc = wave_goto(arm_r, k_wave_hang, -1.0);
    if (rc != 0)
        return rc;
    orch::wave_grip_ratio(false, 1.0);
    orch::wave_grip_ratio(true, 1.0);
    if (hardware_abort_requested())
        return -4;

    for (int round = 1;; ++round)
    {
        std::cout << "[welcome] 第 " << round << " 轮 挥右手\n";
        rc = wave_one(arm_r, true);
        if (rc != 0)
            return rc;
        rc = wave_pause_ms(1000);
        if (rc != 0)
            return rc;
        std::cout << "[welcome] 第 " << round << " 轮 挥左手\n";
        rc = wave_one(arm_l, false);
        if (rc != 0)
            return rc;
        rc = wave_pause_ms(1000);
        if (rc != 0)
            return rc;
    }
}
