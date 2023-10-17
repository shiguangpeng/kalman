/**
* Simple Test program
*/

#define _USE_MATH_DEFINES

#include "Eigen/Dense"
#include <cmath>
#include <iostream>

#define G 9.8
// 时间间隔：单位s/秒
#define T(t) (t)

using namespace Eigen;

int main(int argc, char *argv[]) {
    // 1. 状态预测
    // 时间间隔
    int interval = 1;

    // 状态矩阵
    MatrixXf xhat_(3, 2);
    Matrix3f A(3, 3);
    A << 1, T(interval), T((interval*interval) / 2),
            0, 1, T(interval),
            0, 0, 1;

    Matrix3d Q(3, 3);


    return 0;
}