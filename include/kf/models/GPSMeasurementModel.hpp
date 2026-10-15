#pragma once

#include "kf/core/Types.hpp"

namespace kf
{

class GPSMeasurementModel
{
public:
    static constexpr int kMeasurementDim = 2;

    using MeasurementVector =
        Eigen::Matrix<Scalar, kMeasurementDim, 1>;

    using MeasurementMatrix =
        Eigen::Matrix<Scalar, kMeasurementDim, kStateDim>;

    using MeasurementCovariance =
        Eigen::Matrix<Scalar, kMeasurementDim, kMeasurementDim>;

    MeasurementMatrix measurementMatrix() const;
};

}