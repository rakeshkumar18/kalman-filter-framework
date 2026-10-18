#include "kf/sensors/GPSSensor.hpp"


kf::GPSSensor::GPSSensor(kf::Scalar std_x, kf::Scalar std_y)
: generator_(42),
noise_x_(0.00,std_x),
noise_y_(0.0, std_y)
{

}

kf::MeasurementVector kf::GPSSensor::measure(const kf::StateVector& true_state){
    kf::MeasurementVector z(2);

    z(0) = true_state(0) + noise_x_(generator_);
    z(1) = true_state(1) + noise_y_(generator_);

    return z;
}