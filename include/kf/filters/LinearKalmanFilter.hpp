#pragma once

#include "kf/core/Types.hpp"

namespace kf
{

class LinearKalmanFilter
{

private:
    StateVector state_;
    StateMatrix cov_;

public:
    LinearKalmanFilter(const StateVector &initial_state, const StateMatrix &initial_cov);

    const StateVector& state() const;

    const StateMatrix& covariance() const;

    void predict(const StateMatrix &F, const StateMatrix &Q);

    void update(const MeasurementVector &z, const MeasurementMatrix &H, 
        const MeasurementCovariance &R);

};

}