#pragma once

#include <Eigen/Dense>

using namespace Eigen;

// 定义常规矩阵类型
#define Matrixf(r, c) Eigen::Matrix<float, r, c>
// 定义方阵类型（Square）
#define MatrixSf(d) Eigen::Matrix<float, d, d>

// 状态维度，输入维度，输出维度
template <uint8_t xNum, uint8_t uNum, uint8_t yNum>
class kalmanFilter
{
public:
    MatrixSf(xNum) A = MatrixSf(xNum)::Zero();           // 状态转移矩阵
    Matrixf(xNum, uNum) B = Matrixf(xNum, uNum)::Zero(); // 输入矩阵
    MatrixSf(xNum) Q = MatrixSf(xNum)::Zero();           // 过程噪声协方差矩阵
    MatrixSf(yNum) R = MatrixSf(yNum)::Zero();           // 输入噪声协方差矩阵
    Matrixf(yNum, xNum) H = Matrixf(yNum, xNum)::Zero(); // 输出矩阵
    MatrixSf(xNum) P = MatrixSf(xNum)::Identity();       // 后验估计协方差矩阵
    Matrixf(xNum, 1) x = Matrixf(xNum, 1)::Zero();       // 保留每次的状态
    Matrixf(yNum, 1) y = Matrixf(yNum, 1)::Zero();       // 输出矩阵
public:
    // 构造函数
    kalmanFilter() {}
    // 设置离散状态空间方程，及测量矩阵
    void setFunc(MatrixSf(xNum) _A, Matrixf(xNum, uNum) _B, Matrixf(yNum, xNum) _H, float _Ts = 0)
    {
        if(_Ts<=0)
        {
            this->A = _A;
            this->B = _B;
        }
        else
        {
            // 输入的是连续，需要做离散化
            MatrixSf(xNum) AI = MatrixSf(xNum)::Identity();
            this->A = AI + _Ts * _A;
            this->B = _Ts * _B;
        }
        this->H = _H;
    }
    // 设置协方差矩阵（注意，协方差矩阵可以很小，但不能为零）
    void setConv(MatrixSf(xNum) _Q, MatrixSf(yNum) _R, MatrixSf(xNum) _P = MatrixSf(xNum)::Identity())
    {
        this->Q = _Q;
        this->R = _R;
        this->P = _P;
    }
    // 求解卡尔曼滤波(输入参数为状态估计器的输入，以及观测值输入)
    void f(Matrixf(uNum, 1) _u, Matrixf(yNum, 1) _y)
    {
        // 计算先验状态估计
        Matrixf(xNum, 1) x_minus = A * x + B * _u;
        // 计算先验估计协方差
        MatrixSf(xNum) P_minus = A * P * A.transpose() + Q;
        // 计算卡尔曼增益
        MatrixSf(yNum) temp = H * P_minus * H.transpose() + R;
        Matrixf(xNum, yNum) K = P_minus * H.transpose() * temp.inverse();
        // 更新后验估计
        x = x_minus + K * (_y - H * x_minus);
        // 更新后验估计协方差
        MatrixSf(xNum) E = MatrixSf(xNum)::Identity();
        P = (E - K * H) * P_minus;
        // 计算输出矩阵
        y = H * x;
    }
    Matrixf(yNum, 1) getOut()
    {
        return y;
    }
};