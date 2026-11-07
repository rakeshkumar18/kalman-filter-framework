#pragma once

#include "kf/core/EstimatorSnapshot.hpp"
#include "kf/core/Types.hpp"

#include "kf/models/GPSMeasurementModel.hpp"
#include "kf/models/LidarMeasurementModel.hpp"
#include "kf/models/NonlinearVehicleModel.hpp"

#include "kf/sensors/GPSSensor.hpp"
#include "kf/sensors/LidarSensor.hpp"

#include "kf/filters/ExtendedKalmanFilter.hpp"

namespace kf {

class EKFSimulation {

public:
  //===============================================
  // Constructor
  //===============================================

  explicit EKFSimulation(Scalar dt);

  //===============================================
  // Simulation Control
  //===============================================
  bool step(EstimatorSnapshot &snapshot);

  void reset();

  //===============================================
  // Sensor enable / disable
  //===============================================

  void setGpsEnabled(bool enabled);

  void setLidarEnabled(bool enabled);

  bool gpsEnabled() const;

  bool lidarEnabled() const;

  //===============================================
  // Timing Information
  //===============================================

  Scalar predictionRateHz() const;

  Scalar gpsRateHz() const;

  Scalar lidarRateHz() const;

  //===============================================
  // GPS Information
  //===============================================

  Scalar gpsStdX() const;
  Scalar gpsStdY() const;

  //===============================================
  // Lidar Information
  //===============================================

  Scalar lidarRangeStd() const;
  Scalar lidarBearingStd() const;
  const Landmark &landmark() const;

  //===============================================
  // EKF process noise information
  //===============================================

  Scalar qX() const;
  Scalar qY() const;
  Scalar qV() const;
  Scalar qPsi() const;

private:
  //===============================================
  // Initial State Helpers
  //===============================================
  static StateVector makeInitialTrueState();
  static StateVector makeInitialEstimate();
  static StateMatrix makeInitialCovariance();

  // ========================================================
  // BASE SIMULATION TIMING
  //
  // dt = 0.01 s
  // prediction = 100 Hz
  // ========================================================

  Scalar dt_;
  Scalar simulation_time_ = 0.0;
  int step_ = 0;
  int max_steps_ = 5000;

  // ========================================================
  // SENSOR RATES
  // ========================================================

  Scalar gps_rate_hz_ = 5.0;
  Scalar lidar_rate_hz_ = 20.0;

  Scalar gps_period_ = 1.0 / gps_rate_hz_;
  Scalar lidar_period_ = 1.0 / lidar_rate_hz_;

  Scalar next_gps_time_ = gps_period_;
  Scalar next_lidar_time_ = lidar_period_;

  // ========================================================
  // SENSOR ENABLE FLAGS
  // ========================================================

  bool gps_enabled_ = true;

  bool lidar_enabled_ = true;

  // ========================================================
  // VEHICLE PARAMETERS
  // ========================================================

  VehicleParameters vehicle_parameters_;
  VehicleControlInput control_input_;

  // ========================================================
  // LANDMARK
  // ========================================================

  Landmark landmark_;

  // ========================================================
  // INITIAL CONDITIONS
  // ========================================================

  StateVector initial_true_state_;
  StateVector true_state_;

  StateVector initial_estimate_;
  StateMatrix initial_covariance_;

  // ========================================================
  // PROCESS NOISE
  //
  // State:
  //
  // x = [X, Y, v, psi]
  // ========================================================

  Scalar q_x_ = 0.01;

  Scalar q_y_ = 0.01;

  Scalar q_v_ = 0.02;

  Scalar q_psi_ = 0.005;

  StateMatrix Q_;

  // ========================================================
  // GPS CONFIGURATION
  // ========================================================

  Scalar gps_std_x_ = 1.0;
  Scalar gps_std_y_ = 1.5;

  MeasurementCovariance R_gps_;

  // ========================================================
  // LIDAR CONFIGURATION
  // ========================================================

  Scalar lidar_range_std_ = 0.3;

  Scalar lidar_bearing_std_ = 0.01;

  MeasurementCovariance R_lidar_;

  // ========================================================
  // MODELS
  // ========================================================

  NonlinearVehicleModel vehicle_model_;
  GPSMeasurementModel gps_model_;
  LidarMeasurementModel lidar_model_;

  // ========================================================
  // SENSOR SIMULATORS
  // ========================================================

  GPSSensor gps_sensor_;

  LidarSensor lidar_sensor_;

  // ========================================================
  // FILTER
  // ========================================================

  ExtendedKalmanFilter ekf_;
};

} // namespace kf