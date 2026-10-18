# pragma once
#include <Eigen/Dense>

namespace kf
{
    using Scalar = double;
    constexpr int kStateDim = 4;

    using StateVector = Eigen::Matrix<Scalar, kStateDim, 1>;
    using StateMatrix = Eigen::Matrix<Scalar, kStateDim, kStateDim>;
    using ProcessNoiseMapping = Eigen::Matrix<Scalar, kStateDim, 2>;
    using AccelerationCovariance = Eigen::Matrix<Scalar, 2, 2>;

    using MeasurementVector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
    using MeasurementMatrix = Eigen::Matrix<Scalar, Eigen::Dynamic, kStateDim>;
    using MeasurementCovariance = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    using KalmanGainMatrix = Eigen::Matrix<Scalar,kStateDim,Eigen::Dynamic>;
}