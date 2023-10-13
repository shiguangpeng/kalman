//
// Created by okyousgp on 2023/10/11.
//

#ifndef CIVIC_POSITIONMEASUREMENTMODEL_HPP
#define CIVIC_POSITIONMEASUREMENTMODEL_HPP

namespace KalmanProject {
    namespace Civic {

        // 位置使用：x, y来描述
        template<typename T>
        class PositionMeasurement : public Kalman::Vector<T, 2> {
        public:
            KALMAN_VECTOR(PositionMeasurement, T, 2)

            // x
            static constexpr size_t posX = 0;
            // y
            static constexpr size_t posY = 0;

            // 返回位置：x, y
            T getX() const {
                return this->posX;
            }

            T getY() const {
                return this->posY;
            }

            T &getX() {
                return *(this->posX);
            }

            T &getY() {
                return *(this->posY);
            }
        };

        // 位置量测模型
        template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
        class PositionMeasurementModel
                : public Kalman::LinearizedSystemModel<State<T>, PositionMeasurement<T>, CovarianceBase> {
        public:
            // 状态定义S
            typedef KalmanProject::Civic::State<T> S;
            // 取值M
            typedef KalmanProject::Civic::PositionMeasurement<T> M;
        };
    }
}

#endif //CIVIC_POSITIONMEASUREMENTMODEL_HPP
