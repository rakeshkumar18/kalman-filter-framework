#pragma once

#include "kf/core/Types.hpp"

namespace kf {

class ExtendedKalmanFilter {
private:
  StateVector state_;
  StateMatrix covariance_;

  MeasurementVector innovation_;
  KalmanGainMatrix kalman_gain_;

public:
  ExtendedKalmanFilter(const StateVector &initial_state,
                       const StateMatrix &initial_covariance);

  const StateVector &state() const;

  const StateMatrix &covariance() const;

  const MeasurementVector &innovation() const;

  const KalmanGainMatrix &kalmanGain() const;

  // ========================================================
  // EKF prediction
  //
  // x- = f(x+, u)
  // P- = F P+ F^T + Q
  // ========================================================

  void predict(const StateVector &predicted_state, const StateMatrix &F,
               const StateMatrix &Q);

  void update(const MeasurementVector &innovation, const MeasurementMatrix &H,
              const MeasurementCovariance &R);
};

} // namespace kf