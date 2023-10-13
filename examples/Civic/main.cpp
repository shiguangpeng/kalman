//#define _USE_MATH_DEFINES
//
//#include <cmath>
//
//// 系统方程和量测方程
//#include "SystemModel.hpp"
//#include "PositionMeasurementModel.hpp"
//#include "VelocityMeasurementModel.hpp"
//#include "AccMeasurementModel.hpp"
//
//#include <kalman/ExtendedKalmanFilter.hpp>
//
//#include <iostream>
//#include <random>
//#include <chrono>
//
//using namespace KalmanProject;
//
//typedef float T;
//
//// Some type shortcuts
//typedef Civic::State<T> State;
//typedef Civic::Control<T> Control;
//typedef Civic::SystemModel<T> SystemModel;
//
//typedef Civic::PositionMeasurement<T> PositionMeasurement;
//typedef Civic::PositionMeasurementModel<T> PositionModel;
//typedef Civic::OrientationMeasurement<T> OrientationMeasurement;
//
//int main(int argc, char **argv) {
//    // Simulated (true) system state
//    State x;
//    x.setZero();
//
//    // Control input
//    Control u;
//    // System
//    SystemModel sys;
//
//    // Measurement models
//    // Set position landmarks at (-10, -10) and (30, 75)
//    PositionModel pm(-10, -10, 30, 75);
//    OrientationModel om;
//
//    // Random number generation (for noise simulation)
//    std::default_random_engine generator;
//    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
//    std::normal_distribution<T> noise(0, 1);
//
//    // Some filters for estimation
//    // Pure predictor without measurement updates
//    Kalman::ExtendedKalmanFilter<State> predictor;
//    // Extended Kalman Filter
//    Kalman::ExtendedKalmanFilter<State> ekf;
//    // Unscented Kalman Filter
//    Kalman::UnscentedKalmanFilter<State> ukf(1);
//
//    // Init filters with true system state
//    predictor.init(x);
//    ekf.init(x);
//    ukf.init(x);
//
//    // Standard-Deviation of noise added to all state vector components during state transition
//    T systemNoise = 0.1;
//    // Standard-Deviation of noise added to all measurement vector components in orientation measurements
//    T orientationNoise = 0.025;
//    // Standard-Deviation of noise added to all measurement vector components in distance measurements
//    T distanceNoise = 0.25;
//
//    // Simulate for 100 steps
//    const size_t N = 100;
//    for (size_t i = 1; i <= N; i++) {
//        // Generate some control input
//        u.v() = 1. + std::sin(T(2) * T(M_PI) / T(N));
//        u.dtheta() = std::sin(T(2) * T(M_PI) / T(N)) * (1 - 2 * (i > 50));
//
//        // Simulate system
//        x = sys.f(x, u);
//
//        // Add noise: Our robot move is affected by noise (due to actuator failures)
//        x.x() += systemNoise * noise(generator);
//        x.y() += systemNoise * noise(generator);
//        x.theta() += systemNoise * noise(generator);
//
//        // Predict state for current time-step using the filters
//        auto x_pred = predictor.predict(sys, u);
//        auto x_ekf = ekf.predict(sys, u);
//        auto x_ukf = ukf.predict(sys, u);
//
//        // Orientation measurement
//        {
//            // We can measure the orientation every 5th step
//            OrientationMeasurement orientation = om.h(x);
//
//            // Measurement is affected by noise as well
//            orientation.theta() += orientationNoise * noise(generator);
//
//            // Update EKF
//            x_ekf = ekf.update(om, orientation);
//
//            // Update UKF
//            x_ukf = ukf.update(om, orientation);
//        }
//
//        // Position measurement
//        {
//            // We can measure the position every 10th step
//            PositionMeasurement position = pm.h(x);
//
//            // Measurement is affected by noise as well
//            position.d1() += distanceNoise * noise(generator);
//            position.d2() += distanceNoise * noise(generator);
//
//            // Update EKF
//            x_ekf = ekf.update(pm, position);
//
//            // Update UKF
//            x_ukf = ukf.update(pm, position);
//        }
//
//        // Print to stdout as csv format
//        std::cout << x.x() << "," << x.y() << "," << x.theta() << ","
//                  << x_pred.x() << "," << x_pred.y() << "," << x_pred.theta() << ","
//                  << x_ekf.x() << "," << x_ekf.y() << "," << x_ekf.theta() << ","
//                  << x_ukf.x() << "," << x_ukf.y() << "," << x_ukf.theta()
//                  << std::endl;
//    }
//
//    return 0;
//}


/**
* Simple Test program
*/

#define _USE_MATH_DEFINES

#include "Eigen/Core"
#include <cmath>
#include <iostream>

#define G 9.8

int main(int argc, char *argv[]) {
    // 时间步长
    const int T = 1;
    // 机动频率
    const float alpha = 0.2;
    // 加速度
    const float acc = 1;

    // 状态转移矩阵
    Eigen::Matrix3f F;
    F << 1, T, (1 / alpha * alpha) * (-1 + alpha * T + expf(-alpha * T)),
            0, 1, (1 / alpha) * (1 - expf(-alpha * T)),
            0, 0, expf(-alpha * T);

    // 控制矩阵U
    Eigen::Matrix<float, 3, 2> U;
    U << (1 / acc) * (-T + (acc * T * T) / 2 + (1 - expf(-acc * T) / acc)),
            T - (1 - expf(-acc * T) / acc),
            1 - expf((-acc * T)),
            (1 / acc) * (-T + (acc * T * T) / 2 + (1 - expf(-acc * T) / acc)),
            T - (1 - expf(-acc * T) / acc),
            1 - expf((-acc * T));

    // 状态方程噪声的方差
    float max_acc = 0.5 * G;
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
    Eigen::Matrix<float, 3, 2> x_hat;
    x_hat << 5, 1.2, 0.9, 7, 1.6, 0.7;
    // 1. 状态估计
    Eigen::Matrix<float, 3, 2> x_hat_ = F * x_hat + (U * avg_acc);
    std::cout << x_hat_ << std::endl;

    // 2. 计算协方差
    Eigen::Matrix<float, 3, 3> pk;
    pk << 0.2, 0.6, 0.9, 0.9, 0.25, 0.43, 0.9, 0.25, 0.3;
    Eigen::Matrix<float, 3, 3> p_k = F * pk * F.transpose();

    /***************** 更新/校正 ******************/
    // 3. 卡尔曼增益K
    Eigen::Matrix<float, 2, 3> H;// H为观测矩阵，因为只有位置观测值，因此只有两个
    H << 1, 0, 0, 1, 0, 0;
    Eigen::Matrix<float, 3, 3> Kk = (p_k * H.transpose()) / (H * p_k * H.transpose());

    // 4. 后验估计

    return 0;
}