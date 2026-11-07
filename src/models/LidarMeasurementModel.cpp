#include "kf/models/LidarMeasurementModel.hpp"

#include <cmath>
#include <stdexcept>

kf::LidarMeasurementModel::LidarMeasurementModel(const kf::Landmark &landmark)
    : landmark_(landmark) {}

kf::Scalar kf::LidarMeasurementModel::normalizeAngle(kf::Scalar angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }

  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }

  return angle;
}

kf::MeasurementVector kf::LidarMeasurementModel::predictMeasurement(
    const kf::StateVector &state) const {
  const kf::Scalar X = state(0);

  const kf::Scalar Y = state(1);

  const kf::Scalar psi = state(3);

  const kf::Scalar dx = landmark_.x - X;

  const kf::Scalar dy = landmark_.y - Y;

  const kf::Scalar range = std::sqrt(dx * dx + dy * dy);

  const kf::Scalar bearing = normalizeAngle(std::atan2(dy, dx) - psi);

  kf::MeasurementVector measurement(2);

  measurement(0) = range;

  measurement(1) = bearing;

  return measurement;
}

kf::MeasurementMatrix
kf::LidarMeasurementModel::jacobian(const kf::StateVector &state) const {
  const kf::Scalar X = state(0);

  const kf::Scalar Y = state(1);

  const kf::Scalar dx = landmark_.x - X;

  const kf::Scalar dy = landmark_.y - Y;

  const kf::Scalar q = dx * dx + dy * dy;

  // Avoid singularity when the vehicle is
  // effectively at the landmark location.
  constexpr kf::Scalar minimum_q = 1e-12;

  if (q < minimum_q) {
    throw std::runtime_error("LiDAR Jacobian undefined near landmark");
  }

  const kf::Scalar range = std::sqrt(q);

  kf::MeasurementMatrix H(2, 4);

  H.setZero();

  // Range row
  H(0, 0) = -dx / range;

  H(0, 1) = -dy / range;

  // Bearing row
  H(1, 0) = dy / q;

  H(1, 1) = -dx / q;

  H(1, 3) = -1.0;

  return H;
}

kf::MeasurementVector
kf::LidarMeasurementModel::innovation(const kf::MeasurementVector &measurement,
                                      const kf::StateVector &state) const {
  const kf::MeasurementVector predicted_measurement = predictMeasurement(state);

  kf::MeasurementVector innovation = measurement - predicted_measurement;

  // Bearing innovation must remain inside [-pi, pi].
  innovation(1) = normalizeAngle(innovation(1));
  return innovation;
}