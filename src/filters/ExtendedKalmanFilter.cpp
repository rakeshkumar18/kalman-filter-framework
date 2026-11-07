#include "kf/filters/ExtendedKalmanFilter.hpp"

kf::ExtendedKalmanFilter::ExtendedKalmanFilter(
    const kf::StateVector &initial_state,
    const kf::StateMatrix &initial_covariance)
    : state_(initial_state), covariance_(initial_covariance) {}

const kf::StateVector &kf::ExtendedKalmanFilter::state() const {
  return state_;
}

const kf::StateMatrix &kf::ExtendedKalmanFilter::covariance() const {
  return covariance_;
}

const kf::MeasurementVector &kf::ExtendedKalmanFilter::innovation() const {
  return innovation_;
}

const kf::KalmanGainMatrix &kf::ExtendedKalmanFilter::kalmanGain() const {
  return kalman_gain_;
}

void kf::ExtendedKalmanFilter::predict(const kf::StateVector &predicted_state,
                                       const kf::StateMatrix &F,
                                       const kf::StateMatrix &Q) {
  // ================================================
  // Mean propagation
  //
  // x- = f(x+, u)
  // ================================================

  state_ = predicted_state;

  // ================================================
  // Covariance propagation
  //
  // P- = F P+ F^T + Q
  // ================================================

  covariance_ = F * covariance_ * F.transpose() + Q;
}

void kf::ExtendedKalmanFilter::update(const kf::MeasurementVector &innovation,
                                      const kf::MeasurementMatrix &H,
                                      const kf::MeasurementCovariance &R) {
  // ========================================================
  // Store innovation for diagnostics
  // ========================================================

  innovation_ = innovation;

  // ========================================================
  // Innovation covariance
  //
  // S = H P- H^T + R
  // ========================================================

  kf::MeasurementCovariance S = H * covariance_ * H.transpose() + R;

  // ========================================================
  // Kalman gain
  //
  // K = P- H^T S^-1
  // ========================================================

  kalman_gain_ = covariance_ * H.transpose() * S.inverse();

  // ========================================================
  // State correction
  //
  // x+ = x- + K y
  // ========================================================

  state_ = state_ + kalman_gain_ * innovation_;

  // ========================================================
  // Covariance correction
  //
  // P+ = (I - KH) P-
  // ========================================================

  const kf::StateMatrix I = kf::StateMatrix::Identity();

  covariance_ = (I - kalman_gain_ * H) * covariance_;
}