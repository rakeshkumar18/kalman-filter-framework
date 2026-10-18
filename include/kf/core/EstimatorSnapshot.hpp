#pragma once
#include "kf/core/Types.hpp"

namespace kf
{

struct EstimatorSnapshot
{
    Scalar time;
    StateVector true_state;

    MeasurementVector gps_measurement;

    StateVector predicted_state;
    StateMatrix predicted_covariance;

    StateVector estimated_state;
    StateMatrix estimated_covariance;

    MeasurementVector innovation;
    KalmanGainMatrix kalman_gain;
};


}