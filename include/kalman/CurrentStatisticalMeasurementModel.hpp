//
// Created by okyousgp on 2023/10/19.
//

#ifndef KALMAN_CURRENTSTATISTICALMEASUREMENTMODEL_HPP
#define KALMAN_CURRENTSTATISTICALMEASUREMENTMODEL_HPP

#include "MeasurementModel.hpp"

namespace Kalman
{

/**
 * @brief observation equation extends MeasurementModel\b
 *
 * @tparam T data types
 * @tparam StateType Kalman::Matrix class
 * @tparam MeasurementType Kalman::Matrix class
 * @tparam CovarianceBase covariance class
 * @tparam R Matrix's Row
 * @tparam C Matrix's Col
 */
	template<typename T, class StateType, class MeasurementType, template<class> class CovarianceBase = StandardBase, int R = 2, int C = 3>
	class CurrentStatisticMeasurementModel : public MeasurementModel<StateType, MeasurementType, CovarianceBase>
	{
	 public:
		typedef MeasurementModel<StateType, MeasurementType, CovarianceBase> Base;
		using typename Base::State;
		using typename Base::Measurement;

		Kalman::Matrix<T, R, C> H;
		Kalman::Matrix<T, R, R> V;

	 public:
		CurrentStatisticMeasurementModel() = default;

		/**
		 *
		 * @param H Observation Matrix
		 * @param V Gaussian noise Matrix
		 */
		CurrentStatisticMeasurementModel(const Kalman::Matrix<T, R, C>& H,
				const Kalman::Matrix<T, R, R>& V)
		{
			// this->H << 1, 0, 0, 0, 1, 0;
			// this->V << V.Random(R, C);
			this->H << H;
			this->V << V;
		}

		Measurement h(const State& x) const override
		{
			return this->H * x + this->V;
		}

	 public:
		~CurrentStatisticMeasurementModel() override = default;
	};

}// namespace Kalman


#endif//KALMAN_CURRENTSTATISTICALMEASUREMENTMODEL_HPP
