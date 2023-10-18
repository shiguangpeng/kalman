/**
* Simple Test program
*/

#define _USE_MATH_DEFINES

#include "Eigen/Dense"
#include <cmath>
#include <iostream>

// 重力加速度
#define G (9.8)
// 时间间隔：单位s/秒
#define T(t) (t)
// 机动频率
#define ALPHA(a) (a)
// ALPHA*T
#define A_T(a, t) (ALPHA(a) * T(t))
//#define A_T(a, t) ((a) * (t))
// 指数运算
#define EXP(x) (exp(x))
#define INDEX(x, n) (pow(x, n))
#define MAX_ACC(a) (a)
#define SIGMA_SQR(avg_acc, max_acc) (0 < avg_acc && avg_acc < max_acc ? (4 / EIGEN_PI - 1) * INDEX(MAX_ACC(max_acc) - avg_acc, 2) : (4 / EIGEN_PI - 1) * INDEX(MAX_ACC(-max_acc) + avg_acc, 2))


using namespace std;
using namespace Eigen;

int main(int argc, char *argv[]) {
    /**
     * 可控参数列表
     * */
    // 时间间隔
    int gap_t = 1;
    // 机动频率
    float m_freq = 0.2;
    // 最大加速度
    float max_acc = 2.5 * G;

    Matrix3f A(3, 3);
    A << 1, T(gap_t), 0.5 * (ALPHA(gap_t * gap_t)),
            0, 1, T(gap_t),
            0, 0, 1;

    Vector3f U(3);
    U << (1 / ALPHA(m_freq)) * (-1 * T(gap_t) + 0.5 * (A_T(m_freq, gap_t * gap_t)) + (1 - EXP(-1 * A_T(m_freq, gap_t))) / (ALPHA(m_freq))),
            T(gap_t) - (1 - EXP(-1 * A_T(m_freq, gap_t))) / (ALPHA(m_freq)),
            1 - EXP(-1 * A_T(m_freq, gap_t));

    Matrix3f Q(3, 3);
    float Q_11 = (0.5 * INDEX(ALPHA(m_freq), -5)) * (1 - EXP(-2 * A_T(m_freq, gap_t)) + (2 * A_T(m_freq, gap_t)) + (2 / 3 * (INDEX(A_T(m_freq, gap_t), 3))) - (2 * INDEX(A_T(m_freq, gap_t), 2)) - (4 * A_T(m_freq, gap_t) * EXP(-1 * A_T(m_freq, gap_t))));
    float Q_12 = (0.5 * INDEX(ALPHA(m_freq), -4)) * (EXP(-2 * A_T(m_freq, gap_t)) + 1 - 2 * EXP(-1 * A_T(m_freq, gap_t)) + 2 * A_T(m_freq, gap_t) * EXP(-1 * A_T(m_freq, gap_t)) - 2 * A_T(m_freq, gap_t) + INDEX(A_T(m_freq, gap_t), 2));
    float Q_13 = (0.5 * INDEX(ALPHA(m_freq), -3)) * (1 - EXP(-2 * A_T(m_freq, gap_t)) - 2 * A_T(m_freq, gap_t) * EXP(-1 * A_T(m_freq, gap_t)));
    float Q_22 = (0.5 * INDEX(ALPHA(m_freq), -3)) * (4 * EXP(-1 * A_T(m_freq, gap_t)) - 3 - EXP(-2 * A_T(m_freq, gap_t)) + 2 * A_T(m_freq, gap_t));
    float Q_23 = (0.5 * INDEX(ALPHA(m_freq), -2)) * (EXP(-2 * A_T(m_freq, gap_t)) + 1 - 2 * EXP(-A_T(m_freq, gap_t)));
    float Q_33 = (0.5 * INDEX(ALPHA(m_freq), -1)) * (1 - EXP(-2 * A_T(m_freq, gap_t)));

    Q << Q_11, Q_12, Q_13, Q_12, Q_22, Q_23, Q_13, Q_23, Q_33;

    // 状态矩阵, 先验值
    MatrixXf xhat_(3, 2);
    // x_hat
    MatrixXf x_hat(3, 2);
    x_hat << 0, 0,
            0, 0,
            0, 0;

    // x, y方向的加速度
    RowVectorXf avg_acc(2);
    avg_acc << 1, 1;

    /**
     * 滤波操作
     * */
    // 1. 目标状态先验值/预测
    xhat_ = A * x_hat + U * avg_acc;

    // 2. 状态变量协方差先验值/预测
    Matrix3f phat_(3, 3);
    Matrix3f pk(3, 3);
    pk = pk.Random(3, 3);

    // qk
    Matrix3f qk(3, 3);
    qk = 2 * ALPHA(m_freq) * SIGMA_SQR(avg_acc[0], MAX_ACC(max_acc)) * Q;
    phat_ = A * pk * A.transpose() + qk;

    // H，观测矩阵
    Matrix2Xf H(2, 3);
    H << 1, 0, 0, 0, 1, 0;
    // 3. 滤波增益Kk
    Matrix3Xf Kk(3, 2);
    Matrix2f R(2, 2);
    // 设备噪声协方差矩阵，服从均值为0，方差为R的高斯白噪声，这里是随机初始化。
    R = R.Random(2, 2);
    Kk = phat_ * H.transpose() * (H * phat_ * H.transpose() + R).inverse();

    Matrix2f Z(2, 2);
    Z << 89, 300, 2, 1;

    // 4. 更新状态x_hat是状态
    x_hat = xhat_ + Kk * (Z - H * xhat_);

    // 5. 协方差更新
    Matrix3f I(3, 3);
    pk = (I.Identity() - Kk * H) * phat_;

    //todo: 可实现“机动频率ALPHA和机动加速度的方差SIGMA_SQR可以自适应”的改进CSM模型，使其状态估计更加准确。
    return 0;
}