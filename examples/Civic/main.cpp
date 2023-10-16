/**
* Simple Test program
*/

#define _USE_MATH_DEFINES

#include "Eigen/Dense"
#include <cmath>
#include <iostream>

#define G 9.8

int main(int argc, char *argv[]) {
    // 时间步长
    const int T = 2;
    // 机动频率
    const float alpha = 0.2;
    // 加速度
    const float acc = 0.32*G;

    // 测量值 y, y是4行1列的矩阵
    Eigen::Matrix<float, 4, 1> yk;
    // todo: input
    yk << 30, 2.4, 43, 2.9;

    // 状态转移矩阵
    Eigen::Matrix<float, 6, 6> F;
    F << 1, T, (1 / alpha * alpha) * (-1 + alpha * T + expf(-alpha * T)), 0, 0, 0,
            0, 1, (1 / alpha) * (1 - expf(-alpha * T)), 0, 0, 0,
            0, 0, expf(-alpha * T), 0, 0, 0,
            0, 0, 0, 1, T, (1 / alpha * alpha) * (-1 + alpha * T + expf(-alpha * T)),
            0, 0, 0, 0, 1, (1 / alpha) * (1 - expf(-alpha * T)),
            0, 0, 0, 0, 0, expf(-alpha * T);

    // 控制矩阵U
    Eigen::Matrix<float, 6, 1> U;
    U << (1 / acc) * (-T + (acc * T * T) / 2 + (1 - expf(-acc * T) / acc)),
            T - (1 - expf(-acc * T) / acc),
            1 - expf((-acc * T)),
            (1 / acc) * (-T + (acc * T * T) / 2 + (1 - expf(-acc * T) / acc)),
            T - (1 - expf(-acc * T) / acc),
            1 - expf((-acc * T));

    // 状态方程噪声的方差
    float max_acc = 2 * G;
    float avg_acc = 0.2 * G;

    Eigen::Matrix3f Q;
    Eigen::Matrix3f Q_;

    float q_11 =
            (1 - expf(-2 * alpha * T) + 2 * alpha * T + (2 * powf(alpha, 3) * powf(T, 3)) / 3 -
             2 * powf(alpha, 2) * T * T -
             4 * alpha * T * expf(-alpha * T)) /
            (2 * powf(alpha, 5));
    float q_12 = (expf(-2 * alpha * T) + 1 - 2 * expf(-alpha * T) + 2 * alpha * T * expf(-alpha) - 2 * alpha * T +
                  pow(alpha, 2) * T * T) /
                 (2 * powf(alpha, 4));
    float q_13 = (1 - expf(-2 * alpha * T) - 2 * alpha * T * expf(-alpha * T)) / (2 * powf(alpha, 3));
    float q_22 = (4 * expf(-alpha * T) - 3 - expf(-2 * alpha * T) + 2 * alpha * T) / (2 * powf(alpha, 3));
    float q_23 = (expf(-2 * alpha * T) + 1 - 2 * expf(-alpha * T)) / (2 * powf(alpha, 2));
    float q_33 = (1 - expf(-2 * alpha * T)) / (2 * alpha * T);

    Q_ << q_11, q_12, q_13,
            q_12, q_22, q_23,
            q_13, q_23, q_33;

    float square_theta;
    if (avg_acc >= 0) {
        square_theta = (4 - EIGEN_PI) / EIGEN_PI * powf((max_acc - avg_acc), 2);
    } else {
        square_theta = (4 - EIGEN_PI) / EIGEN_PI * powf((-max_acc + avg_acc), 2);
    }
    Q << 2 * alpha * square_theta * Q_;

    /** 卡尔曼滤波步骤 */

    /****************** 预测/估计 ******************/
    Eigen::Matrix<float, 6, 1> x_hat;
    x_hat << 5, 1.2, 0.9, 7, 1.6, 0.7;
    // 1. 状态估计
    Eigen::Matrix<float, 6, 1> x_hat_ = F * x_hat + (U * avg_acc);

    // 2. 计算协方差
    Eigen::Matrix<float, 6, 6> temp = Eigen::Matrix<float, 6, 6>::Random(6, 6);
    // 模拟生成协方差矩阵
    Eigen::Matrix<float, 6, 6> pk = temp * temp.transpose();
    Eigen::Matrix<float, 6, 6> pk_ = F * pk * F.transpose();

    /***************** 更新/校正 ******************/
    // 3. 卡尔曼增益K,
    Eigen::Matrix<float, 4, 6> H;   // H是观测矩阵，这里观测了4个状态，分别是横纵向位置和速度
    H << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0;

    Eigen::Matrix<float, 6, 4> Kk = (pk_ * H.transpose()) * (H * pk_ * H.transpose()).inverse();

    // 4. 观测值的后验估计
    x_hat = x_hat_ + Kk * (yk - H * x_hat_);

    // 5. 后验概率
    Eigen::MatrixXf I;
    pk = (I.setIdentity(6, 6) - Kk * H) * pk_;

    std::cout << "x_hat=\n" << x_hat << "\n"
              << "pk=\n" << pk << std::endl;

    return 0;
}