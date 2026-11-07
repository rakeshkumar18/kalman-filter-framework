#pragma once

#include <random>

#include "kf/core/Types.hpp"
#include "kf/models/LidarMeasurementModel.hpp"

namespace kf {

class LidarSensor {
public:
  LidarSensor(const Landmark &landmark, Scalar range_std, Scalar bearing_std);

  MeasurementVector measure(const StateVector &true_state);

private:
  LidarMeasurementModel measurement_model_;

  std::mt19937 generator_;

  std::normal_distribution<Scalar> range_noise_;

  std::normal_distribution<Scalar> bearing_noise_;

  static Scalar normalizeAngle(Scalar angle);
};

} // namespace kf