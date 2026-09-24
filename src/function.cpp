
#include <iostream>
#include <fstream>
#include <iomanip>

#include "function.h"

#include "Ti5_socketcan.h"

#include <chrono>
#include <thread>
#include <algorithm>
#include <cmath>
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
        for (int j = 0; j < dof; ++j)
            motor_cmd[j] = rad_to_encoder_cnt(traj(i, j));
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
        tool << 0.23, 0, 0, 0, 0, 0;
    else
        tool << 0.23, 0, 0, 0, 0, 0;
    return tool;
}

} // namespace

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
    m.tool << 0.230, 0, 0, 0, 0, 0;
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

int arm_line_move(Robot_Arm &arm, Matrix<double, 1, 6> &pos, double cart_linear_velocity)
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
    double cart_linear_velocity)
{
    ArmLineMoveResult out;
    std::thread th_r([&]() { out.ret_r = arm_line_move(arm_r, pos_r, cart_linear_velocity); });
    std::thread th_l([&]() { out.ret_l = arm_line_move(arm_l, pos_l, cart_linear_velocity); });
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
    const double cart_linear_velocity)
{
    if (move_r && move_l)
        return arm_dual_line_move(arm_r, pos_r, arm_l, pos_l, cart_linear_velocity);

    ArmLineMoveResult out;
    if (move_r)
        out.ret_r = arm_line_move(arm_r, pos_r, cart_linear_velocity);
    if (move_l)
        out.ret_l = arm_line_move(arm_l, pos_l, cart_linear_velocity);
    return out;
}

namespace
{

constexpr int kBezierArcLutSteps = 400;
constexpr double kBezierMinGuideHeightM = 0.01;
constexpr double kBezierMinVelocityMps = 0.01;
constexpr double kBezierMinDurationS = 0.50;
constexpr double kBezierKeyDsM = 0.003;
constexpr double kBezierKeyDsMinM = 0.002;
constexpr double kBezierKeyDsMaxM = 0.004;
constexpr int kBezierKeyMax = 200;
constexpr double kBezierSkipStartIkM = 0.008;
constexpr double kBezierMaxKeyJointRad = 20.0 * M_PI / 180.0;
constexpr double kBezierIkFkPositionToleranceM = 0.001;
constexpr double kBezierIkFkOrientationToleranceRad = 0.5 * M_PI / 180.0;
constexpr double kBezierKeyFkPositionToleranceM = 0.003;
constexpr double kBezierKeyFkOrientationToleranceRad = 2.0 * M_PI / 180.0;
constexpr int kBezierNoReturnTrajectory = -20;
constexpr int kBezierReturnStartMismatch = -21;

struct PreparedBezierTrajectory
{
    int hand = 0;
    bool skip = false;
    Eigen::MatrixXd q;
    Matrix<double, 1, 6> a = Matrix<double, 1, 6>::Zero();
    Matrix<double, 1, 6> b = Matrix<double, 1, 6>::Zero();
    Matrix<double, 1, 6> c = Matrix<double, 1, 6>::Zero();
    double length_m = 0.0;
    double duration_s = 0.0;
};

std::mutex g_bezier_cache_mu;
Eigen::MatrixXd g_last_bezier_q[2];
bool g_last_bezier_valid[2] = {false, false};

double quintic_s01(double r)
{
    r = std::clamp(r, 0.0, 1.0);
    const double r2 = r * r;
    return r2 * r * (10.0 - 15.0 * r + 6.0 * r2);
}

Matrix<double, 1, 3> bezier_rpy_slerp(
    const Matrix<double, 1, 6> &a,
    const Matrix<double, 1, 6> &b,
    double t)
{
    t = std::clamp(t, 0.0, 1.0);
    Matrix<double, 1, 3> ra, rb;
    ra << a(3), a(4), a(5);
    rb << b(3), b(4), b(5);
    const Matrix4d Ta = Rot_zyx(ra);
    const Matrix4d Tb = Rot_zyx(rb);
    Eigen::Quaterniond qa(Ta.block<3, 3>(0, 0));
    Eigen::Quaterniond qb(Tb.block<3, 3>(0, 0));
    qa.normalize();
    qb.normalize();
    if (qa.dot(qb) < 0.0)
        qb.coeffs() *= -1.0;
    const Eigen::Quaterniond qm = qa.slerp(t, qb);
    const Eigen::Vector3d e = Quaterniond2EulerAngles(qm);
    Matrix<double, 1, 3> out;
    out << e(0), e(1), e(2);
    return out;
}

double wrap_pi_bezier(double a)
{
    while (a > M_PI)
        a -= 2.0 * M_PI;
    while (a < -M_PI)
        a += 2.0 * M_PI;
    return a;
}

double bezier_angle_abs_diff(double a, double b)
{
    return std::abs(wrap_pi_bezier(a - b));
}

bool bezier_rpy_close(
    const Matrix<double, 1, 6> &a,
    const Matrix<double, 1, 6> &b,
    double tol_rad)
{
    return bezier_angle_abs_diff(a(3), b(3)) <= tol_rad &&
           bezier_angle_abs_diff(a(4), b(4)) <= tol_rad &&
           bezier_angle_abs_diff(a(5), b(5)) <= tol_rad;
}

void clear_bezier_cache(int hand)
{
    if (hand < 0 || hand > 1)
        return;
    std::lock_guard<std::mutex> lock(g_bezier_cache_mu);
    g_last_bezier_valid[hand] = false;
    g_last_bezier_q[hand].resize(0, 0);
}

/** 关键点 IK：先锁当前 J2 解析，失败再数值。不在 5 ms 尺度上卡 0.375°。 */
int solve_bezier_ik(
    Robot_Arm &arm,
    const Matrix<double, 1, 6> &pose,
    Eigen::RowVectorXd &q,
    double max_joint_step_rad,
    double fk_pos_tol_m,
    double fk_ori_tol_rad)
{
    Eigen::RowVectorXd q_try = q;
    const double j2 = (q.size() >= 2) ? q(1) : 0.0;
    int rc = arm.Inverse_Kinematics_Analytic(pose, q_try, j2);
    if (rc != 0)
    {
        q_try = q;
        Matrix<double, 1, 6> pose_for_ik = pose;
        rc = arm.Inverse_Kinematics(pose_for_ik, q_try);
    }
    if (rc != 0)
        return rc;
    if (!q_try.allFinite())
        return -10;
    if (arm.Check_Joint_Limit(q_try))
        return -11;
    if ((q_try - q).cwiseAbs().maxCoeff() > max_joint_step_rad)
        return -12;

    const Matrix4d fk_t = arm.Forward_Kinematics(q_try);
    const Matrix<double, 1, 6> fk = T2PosEulerAngles(fk_t);
    if ((fk.head<3>() - pose.head<3>()).norm() > fk_pos_tol_m)
        return -13;
    const Matrix4d goal_t = TR(pose);
    const Matrix3d r_err = goal_t.block<3, 3>(0, 0).transpose() * fk_t.block<3, 3>(0, 0);
    const double angle_err = std::acos(std::clamp((r_err.trace() - 1.0) * 0.5, -1.0, 1.0));
    if (angle_err > fk_ori_tol_rad)
        return -13;

    q = q_try;
    return 0;
}

Eigen::Vector3d quadratic_bezier_xyz(
    const Eigen::Vector3d &a,
    const Eigen::Vector3d &b,
    const Eigen::Vector3d &c,
    double u)
{
    u = std::clamp(u, 0.0, 1.0);
    const double omu = 1.0 - u;
    return omu * omu * a + 2.0 * omu * u * b + u * u * c;
}

int prepare_quadratic_bezier(
    Robot_Arm &arm,
    const Matrix<double, 1, 6> &final_goal,
    double guide_height_m,
    double cart_max_velocity,
    double orient_finish_ratio,
    PreparedBezierTrajectory &out)
{
    out = {};
    out.hand = arm_hand_side(arm);
    clear_bezier_cache(out.hand);

    if (out.hand == 1 && right_arm_motors_locked())
    {
        out.skip = true;
        return 0;
    }
    if (!final_goal.allFinite() || !std::isfinite(guide_height_m) ||
        !std::isfinite(cart_max_velocity) || !std::isfinite(orient_finish_ratio))
        return -10;
    if (guide_height_m < kBezierMinGuideHeightM)
        return -15;

    Eigen::MatrixXd q0 = arm_read_motor_joints(out.hand);
    if (hardware_abort_requested())
        return -4;
    if (can_io_faulted())
        return -2;
    if (q0.rows() != 1 || q0.cols() < 6 || !q0.allFinite())
        return -10;

    out.a = T2PosEulerAngles(arm.Forward_Kinematics(q0.row(0)));
    out.c = final_goal;
    out.b = final_goal;
    out.b(2) = final_goal(2) + guide_height_m;
    if (out.b(2) < g_move_cfg.grasp_valid.z_min || out.b(2) > g_move_cfg.grasp_valid.z_max)
        return -17;

    const Eigen::Vector3d pa = out.a.head<3>().transpose();
    const Eigen::Vector3d pb = out.b.head<3>().transpose();
    const Eigen::Vector3d pc = out.c.head<3>().transpose();

    std::vector<double> arc_u(kBezierArcLutSteps + 1, 0.0);
    std::vector<double> arc_s(kBezierArcLutSteps + 1, 0.0);
    Eigen::Vector3d prev = pa;
    for (int i = 1; i <= kBezierArcLutSteps; ++i)
    {
        const double u = static_cast<double>(i) / static_cast<double>(kBezierArcLutSteps);
        const Eigen::Vector3d p = quadratic_bezier_xyz(pa, pb, pc, u);
        arc_u[i] = u;
        arc_s[i] = arc_s[i - 1] + (p - prev).norm();
        prev = p;
    }
    out.length_m = arc_s.back();
    if (out.length_m < 1e-4)
        return -16;

    const double vmax = std::max(kBezierMinVelocityMps, cart_max_velocity);
    out.duration_s = std::max(kBezierMinDurationS, 1.875 * out.length_m / vmax);
    const int n_step = std::max(2, static_cast<int>(std::ceil(out.duration_s / k_arm_traj_dt)));

    int n_seg = std::max(1, static_cast<int>(std::lround(out.length_m / kBezierKeyDsM)));
    n_seg = std::min(n_seg, kBezierKeyMax);
    while (n_seg > 1 && out.length_m / static_cast<double>(n_seg) < kBezierKeyDsMinM)
        --n_seg;
    while (n_seg < kBezierKeyMax && out.length_m / static_cast<double>(n_seg) > kBezierKeyDsMaxM)
        ++n_seg;
    if (n_seg < 100 && out.length_m / 100.0 >= kBezierKeyDsMinM)
        n_seg = 100;
    const int n_knots = n_seg + 1;
    const double key_ds = out.length_m / static_cast<double>(n_seg);

    const double orient_hold = std::clamp(orient_finish_ratio, 0.10, 0.95);
    const bool hold_rpy = bezier_rpy_close(out.a, out.c, 2.0 * M_PI / 180.0);

    auto u_at_arc_fraction = [&](double f) {
        const double target = std::clamp(f, 0.0, 1.0) * out.length_m;
        const auto it = std::lower_bound(arc_s.begin(), arc_s.end(), target);
        if (it == arc_s.begin())
            return 0.0;
        if (it == arc_s.end())
            return 1.0;
        const size_t i1 = static_cast<size_t>(it - arc_s.begin());
        const size_t i0 = i1 - 1;
        const double den = std::max(1e-12, arc_s[i1] - arc_s[i0]);
        const double t = (target - arc_s[i0]) / den;
        return arc_u[i0] + t * (arc_u[i1] - arc_u[i0]);
    };

    auto pose_at_path_fraction = [&](double path_fraction, bool force_c) {
        Matrix<double, 1, 6> pose;
        if (force_c)
            return out.c;
        const double u = u_at_arc_fraction(path_fraction);
        const Eigen::Vector3d p = quadratic_bezier_xyz(pa, pb, pc, u);
        Matrix<double, 1, 3> rpy;
        if (hold_rpy)
            rpy << out.c(3), out.c(4), out.c(5);
        else if (path_fraction <= orient_hold)
            rpy << out.a(3), out.a(4), out.a(5);
        else
        {
            const double t =
                (path_fraction - orient_hold) / std::max(1e-6, 1.0 - orient_hold);
            rpy = bezier_rpy_slerp(out.a, out.c, quintic_s01(t));
        }
        pose << p.x(), p.y(), p.z(), rpy(0), rpy(1), rpy(2);
        return pose;
    };

    std::vector<double> knot_s(static_cast<size_t>(n_knots), 0.0);
    std::vector<Eigen::RowVectorXd> knot_q(static_cast<size_t>(n_knots), q0.row(0));
    Eigen::RowVectorXd q = q0.row(0);
    knot_s[0] = 0.0;
    knot_q[0] = q;

    for (int k = 1; k < n_knots; ++k)
    {
        const bool last = (k + 1 == n_knots);
        const double s = last ? out.length_m : static_cast<double>(k) * key_ds;
        knot_s[static_cast<size_t>(k)] = s;
        if (!last && s < kBezierSkipStartIkM)
        {
            knot_q[static_cast<size_t>(k)] = q;
            continue;
        }

        const double path_fraction = s / out.length_m;
        const Matrix<double, 1, 6> pose = pose_at_path_fraction(path_fraction, last);
        const int rc = solve_bezier_ik(
            arm,
            pose,
            q,
            kBezierMaxKeyJointRad,
            last ? kBezierIkFkPositionToleranceM : kBezierKeyFkPositionToleranceM,
            last ? kBezierIkFkOrientationToleranceRad : kBezierKeyFkOrientationToleranceRad);
        if (rc != 0)
        {
            append_line_trajectory_fail_debug(
                arm_hand_label(out.hand), rc, pose, "quadratic Bezier A-B-C", vmax);
            return rc;
        }
        knot_q[static_cast<size_t>(k)] = q;
    }

    out.q.resize(n_step + 1, q0.cols());
    out.q.row(0) = q0.row(0);
    for (int i = 1; i <= n_step; ++i)
    {
        const double path_fraction =
            (i == n_step) ? 1.0 : quintic_s01(static_cast<double>(i) / static_cast<double>(n_step));
        const double s = path_fraction * out.length_m;
        auto it = std::lower_bound(knot_s.begin(), knot_s.end(), s);
        size_t k1 = static_cast<size_t>(it - knot_s.begin());
        if (k1 == 0)
        {
            out.q.row(i) = knot_q[0];
            continue;
        }
        if (k1 >= knot_s.size())
        {
            out.q.row(i) = knot_q.back();
            continue;
        }
        const size_t k0 = k1 - 1;
        const double den = std::max(1e-12, knot_s[k1] - knot_s[k0]);
        const double t = std::clamp((s - knot_s[k0]) / den, 0.0, 1.0);
        out.q.row(i) = knot_q[k0] + t * (knot_q[k1] - knot_q[k0]);
    }
    out.q.row(n_step) = knot_q.back();

    const Matrix<double, 1, 6> end_fk =
        T2PosEulerAngles(arm.Forward_Kinematics(out.q.row(n_step)));
    const double end_xyz_error = (end_fk.head<3>() - out.c.head<3>()).norm();
    if (end_xyz_error > kBezierIkFkPositionToleranceM)
        return -13;

    std::cout << std::fixed << std::setprecision(4)
              << "[arm] Bezier " << (out.hand == 1 ? "右" : "左")
              << " A=(" << out.a(0) << ',' << out.a(1) << ',' << out.a(2) << ')'
              << " B=(" << out.b(0) << ',' << out.b(1) << ',' << out.b(2) << ')'
              << " C=(" << out.c(0) << ',' << out.c(1) << ',' << out.c(2) << ')'
              << " length=" << out.length_m << "m Tf=" << out.duration_s
              << "s vmax=" << vmax << "m/s keys=" << n_knots
              << " ds=" << key_ds << "m orient_hold=" << orient_hold << '\n';
    return 0;
}

int execute_prepared_bezier(PreparedBezierTrajectory &plan)
{
    if (plan.skip)
        return 0;
    const int rc = execute_arm_trajectory(plan.q, plan.hand, k_arm_traj_dt);
    if (rc != 0)
        return rc;
    {
        std::lock_guard<std::mutex> lock(g_bezier_cache_mu);
        g_last_bezier_q[plan.hand] = plan.q;
        g_last_bezier_valid[plan.hand] = true;
    }
    return 0;
}

int load_reversed_bezier(Robot_Arm &arm, Eigen::MatrixXd &reversed)
{
    const int hand = arm_hand_side(arm);
    Eigen::MatrixXd forward;
    {
        std::lock_guard<std::mutex> lock(g_bezier_cache_mu);
        if (!g_last_bezier_valid[hand] || g_last_bezier_q[hand].rows() < 2)
            return kBezierNoReturnTrajectory;
        forward = g_last_bezier_q[hand];
    }

    const Eigen::MatrixXd q_now = arm_read_motor_joints(hand);
    if (q_now.rows() != 1 || q_now.cols() != forward.cols() ||
        (q_now.row(0) - forward.row(forward.rows() - 1)).cwiseAbs().maxCoeff() > 0.05)
        return kBezierReturnStartMismatch;

    reversed.resize(forward.rows(), forward.cols());
    for (Eigen::Index i = 0; i < forward.rows(); ++i)
        reversed.row(i) = forward.row(forward.rows() - 1 - i);
    return 0;
}

} // namespace

int arm_quadratic_bezier_move(
    Robot_Arm &arm,
    const Matrix<double, 1, 6> &final_goal,
    double guide_height_m,
    double cart_max_velocity,
    double orient_finish_ratio)
{
    PreparedBezierTrajectory plan;
    const int rc = prepare_quadratic_bezier(
        arm, final_goal, guide_height_m, cart_max_velocity, orient_finish_ratio, plan);
    if (rc != 0)
        return rc;
    const int exec = execute_prepared_bezier(plan);
    if (exec == 0)
        record_arm_cartesian_arrival(arm, final_goal, "Bezier C");
    return exec;
}

ArmLineMoveResult arm_dual_quadratic_bezier_move_selective(
    Robot_Arm &arm_r,
    const Matrix<double, 1, 6> &final_r,
    const bool move_r,
    Robot_Arm &arm_l,
    const Matrix<double, 1, 6> &final_l,
    const bool move_l,
    const double guide_height_m,
    const double cart_max_velocity,
    const double orient_finish_ratio)
{
    ArmLineMoveResult out;
    PreparedBezierTrajectory plan_r, plan_l;
    if (move_r)
        out.ret_r = prepare_quadratic_bezier(
            arm_r, final_r, guide_height_m, cart_max_velocity, orient_finish_ratio, plan_r);
    if (move_l)
        out.ret_l = prepare_quadratic_bezier(
            arm_l, final_l, guide_height_m, cart_max_velocity, orient_finish_ratio, plan_l);

    // 双臂必须全部规划成功后才允许任一侧开始运动。
    if ((move_r && out.ret_r != 0) || (move_l && out.ret_l != 0))
    {
        if (move_r && out.ret_r == 0)
            out.ret_r = -14;
        if (move_l && out.ret_l == 0)
            out.ret_l = -14;
        return out;
    }

    if (move_r && move_l)
    {
        std::thread th_r([&]() { out.ret_r = execute_prepared_bezier(plan_r); });
        std::thread th_l([&]() { out.ret_l = execute_prepared_bezier(plan_l); });
        th_r.join();
        th_l.join();
        if (out.ret_r == 0)
            record_arm_cartesian_arrival(arm_r, final_r, "Bezier C");
        if (out.ret_l == 0)
            record_arm_cartesian_arrival(arm_l, final_l, "Bezier C");
        return out;
    }
    if (move_r)
        out.ret_r = execute_prepared_bezier(plan_r);
    if (move_l)
        out.ret_l = execute_prepared_bezier(plan_l);
    if (move_r && out.ret_r == 0)
        record_arm_cartesian_arrival(arm_r, final_r, "Bezier C");
    if (move_l && out.ret_l == 0)
        record_arm_cartesian_arrival(arm_l, final_l, "Bezier C");
    return out;
}

int arm_reverse_last_quadratic_bezier(Robot_Arm &arm)
{
    const int hand = arm_hand_side(arm);
    if (hand == 1 && right_arm_motors_locked())
        return 0;
    Eigen::MatrixXd reversed;
    const int load_rc = load_reversed_bezier(arm, reversed);
    if (load_rc != 0)
        return load_rc;
    const int rc = execute_arm_trajectory(reversed, hand, k_arm_traj_dt);
    if (rc == 0)
        clear_bezier_cache(hand);
    return rc;
}

ArmLineMoveResult arm_dual_reverse_last_quadratic_bezier_selective(
    Robot_Arm &arm_r,
    const bool move_r,
    Robot_Arm &arm_l,
    const bool move_l)
{
    ArmLineMoveResult out;
    Eigen::MatrixXd reverse_r, reverse_l;
    if (move_r)
        out.ret_r = load_reversed_bezier(arm_r, reverse_r);
    if (move_l)
        out.ret_l = load_reversed_bezier(arm_l, reverse_l);
    if ((move_r && out.ret_r != 0) || (move_l && out.ret_l != 0))
    {
        if (move_r && out.ret_r == 0)
            out.ret_r = -14;
        if (move_l && out.ret_l == 0)
            out.ret_l = -14;
        return out;
    }

    if (move_r && move_l)
    {
        std::thread th_r([&]() { out.ret_r = execute_arm_trajectory(reverse_r, 1, k_arm_traj_dt); });
        std::thread th_l([&]() { out.ret_l = execute_arm_trajectory(reverse_l, 0, k_arm_traj_dt); });
        th_r.join();
        th_l.join();
    }
    else if (move_r)
        out.ret_r = execute_arm_trajectory(reverse_r, 1, k_arm_traj_dt);
    else if (move_l)
        out.ret_l = execute_arm_trajectory(reverse_l, 0, k_arm_traj_dt);

    if (move_r && out.ret_r == 0)
        clear_bezier_cache(1);
    if (move_l && out.ret_l == 0)
        clear_bezier_cache(0);
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

void record_arm_cartesian_arrival(
    Robot_Arm &arm,
    const Eigen::Matrix<double, 1, 6> &goal,
    const char *stage)
{
    const int hand = arm_hand_side(arm);
    if (hand == 1 && right_arm_motors_locked())
        return;

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
        is_right, !is_right, 15.0, 6, stage_txt.c_str());
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

Matrix<double, 1, 6> arm_get_base_visual_pos(Robot_Arm &arm, Matrix<double, 1, 6> &falan_visualPos)
{
    const Eigen::Matrix4d T_tcp = arm_fk_tcp_from_encoders(arm);
    const Matrix<double, 1, 6> flange_tool = arm_flange_tool(arm);

    // cout << "==================================================== " << endl;

    // cout << "T_tcp: " << T_tcp << endl;
    // cout << "flange_tool: " << flange_tool << endl;
    // cout << "falan_visualPos: " << falan_visualPos << endl;
    // cout << "==================================================== " << endl;

    



    const Eigen::Matrix4d T_base_visual = T_tcp * TR(flange_tool).inverse() * TR(falan_visualPos);
    return T2PosEulerAngles(T_base_visual);
}
