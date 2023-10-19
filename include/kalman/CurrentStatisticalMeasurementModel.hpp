//
// Created by okyousgp on 2023/10/19.
//

#ifndef KALMAN_CURRENTSTATISTICALMEASUREMENTMODEL_HPP
#define KALMAN_CURRENTSTATISTICALMEASUREMENTMODEL_HPP

#include "MeasurementModel.hpp"


namespace Kalman {

    /**继承MeasurementModel，实现量测模型*/
    template<typename T, class StateType, class MeasurementType, template<class> class CovarianceBase = StandardBase, int R=2, int C=3>
    class CurrentStatisticMeasurementModel : public MeasurementModel<StateType, MeasurementType, CovarianceBase> {
    public:
        typedef MeasurementModel<StateType, MeasurementType, CovarianceBase> Base;
        using typename Base::State;
        using typename Base::Measurement;
        Kalman::Matrix<T, R, C> H;
        Kalman::Matrix<T, R, C> V;

    public:
        // 初始化
        CurrentStatisticMeasurementModel() {
            this->H << 1, 0, 0, 0, 1, 0;
            this->V << V.Random(R, C);
        }
        // 重写virtual方法，产生观测矩阵，特性是Kalman::Vector
        Measurement h(const State &x) const override {

            return H * x + V;
        }

    protected:
        // todo:
        ~CurrentStatisticMeasurementModel() override = default;
    };

}// namespace Kalman


#endif//KALMAN_CURRENTSTATISTICALMEASUREMENTMODEL_HPP
