#include "kf/sensors/LidarSensor.hpp"

#include <cmath>

kf::LidarSensor::LidarSensor(const kf::Landmark &landmark, kf::Scalar range_std,
                             kf::Scalar bearing_std)
    : measurement_model_(landmark), generator_(42),
      range_noise_(0.0, range_std), bearing_noise_(0.0, bearing_std) {}

kf::Scalar kf::LidarSensor::normalizeAngle(kf::Scalar angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }

  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }

  return angle;
}

kf::MeasurementVector
kf::LidarSensor::measure(const kf::StateVector &true_state) {
  // Perfect measurement from ground truth
  kf::MeasurementVector measurement =
      measurement_model_.predictMeasurement(true_state);

  // Add independent Gaussian sensor noise

  measurement(0) += range_noise_(generator_);

  measurement(1) += bearing_noise_(generator_);

  // Bearing must stay inside [-pi, pi]

  measurement(1) = normalizeAngle(measurement(1));

  return measurement;
}