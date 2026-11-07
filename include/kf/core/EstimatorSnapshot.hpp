#pragma once

#include "kf/core/Types.hpp"

namespace kf {

struct EstimatorSnapshot {
  // ========================================================
  // SIMULATION
  // ========================================================

  Scalar time = 0.0;

  StateVector true_state;

  // ========================================================
  // FILTER PRIOR
  // ========================================================

  StateVector predicted_state;

  StateMatrix predicted_covariance;

  // ========================================================
  // FILTER POSTERIOR
  // ========================================================

  StateVector estimated_state;

  StateMatrix estimated_covariance;

  // ========================================================
  // SENSOR STATUS
  // ========================================================

  bool gps_enabled = false;

  bool lidar_enabled = false;

  bool gps_updated = false;

  bool lidar_updated = false;

  // ========================================================
  // GPS
  // ========================================================

  MeasurementVector gps_measurement;

  MeasurementVector gps_innovation;

  KalmanGainMatrix gps_kalman_gain;

  // ========================================================
  // LIDAR
  // ========================================================

  MeasurementVector lidar_measurement;

  MeasurementVector lidar_innovation;

  KalmanGainMatrix lidar_kalman_gain;

  // ========================================================
  // LEGACY / GENERAL DIAGNOSTICS
  //
  // Keep these temporarily because the existing viewer
  // still uses:
  //
  //     snapshot.innovation
  //     snapshot.kalman_gain
  //
  // For LKF:
  //     contains GPS update diagnostics.
  //
  // For EKF:
  //     contains diagnostics from the most recent sensor
  //     update performed during the current filter step.
  //
  // We will remove these after RealTimeViewer becomes fully
  // multi-sensor aware.
  // ========================================================

  MeasurementVector innovation;

  KalmanGainMatrix kalman_gain;
};

} // namespace kf