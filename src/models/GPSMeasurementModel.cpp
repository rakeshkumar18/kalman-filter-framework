#include "kf/models/GPSMeasurementModel.hpp"



kf::GPSMeasurementModel::MeasurementMatrix kf::GPSMeasurementModel::measurementMatrix () const
{
    MeasurementMatrix H = MeasurementMatrix::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    return H;
}
