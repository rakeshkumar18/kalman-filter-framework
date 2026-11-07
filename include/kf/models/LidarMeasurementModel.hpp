#pragma once

#include "kf/core/Types.hpp"

namespace kf {

struct Landmark {
  Scalar x;
  Scalar y;
};

class LidarMeasurementModel {
public:
  explicit LidarMeasurementModel(const Landmark &landmark);

  // ========================================================
  // Nonlinear measurement prediction
  //
  // z_hat = h(x)
  //
  // z(0) = range
  // z(1) = bearing
  // ========================================================

  MeasurementVector predictMeasurement(const StateVector &state) const;

  // ========================================================
  // Measurement Jacobian
  //
  // H = dh/dx
  // ========================================================

  MeasurementMatrix jacobian(const StateVector &state) const;

  MeasurementVector innovation(const MeasurementVector &measurement,
                               const StateVector &state) const;

private:
  Landmark landmark_;

  static Scalar normalizeAngle(Scalar angle);
};

} // namespace kf