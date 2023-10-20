//
// Created by okyousgp on 2023/10/20.
//

#include "TestHelper.h"

#include <kalman/CurrentStatisticalMeasurementModel.hpp>

using namespace Kalman;

typedef float T;
using StateType = Kalman::Matrix<T, 3, 2>;
using Measurement = Kalman::Matrix<T, 2, 2>;

TEST(CurrentStatisticMeasurementModel, h)
{
	Kalman::Matrix<T, 2, 3> H(2, 3);
	H << 1, 0, 0, 0, 1, 0;
	Kalman::Matrix<T, 2, 2> V(2, 2);
	V = V.Random(2, 2);
	CurrentStatisticMeasurementModel<T, StateType, Measurement> csm(H, V);
	StateType x(3, 2);
	x << 3, 4, 2, 1, 1, 3;
	Measurement m = csm.h(x);
	ASSERT_EQ(m.cols(), V.cols());
}