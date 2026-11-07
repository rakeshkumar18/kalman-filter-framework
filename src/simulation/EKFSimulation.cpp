#include "kf/simulation/EKFSimulation.hpp"

kf::EKFSimulation::EKFSimulation(kf::Scalar dt)
    : // ========================================================
      // BASE TIMING
      // ========================================================

      dt_(dt),

      // ========================================================
      // VEHICLE CONFIGURATION
      //
      // lf = distance CG -> front axle
      // lr = distance CG -> rear axle
      // ========================================================

      vehicle_parameters_{1.2, 1.6},

      // ========================================================
      // CONTROL INPUT
      //
      // [ acceleration, steering angle ]
      // ========================================================

      control_input_{0.05, 0.04},

      // ========================================================
      // LIDAR LANDMARK
      // ========================================================

      landmark_{20.0, 10.0},

      // ========================================================
      // INITIAL STATES
      // ========================================================

      initial_true_state_(makeInitialTrueState()),

      true_state_(initial_true_state_),

      initial_estimate_(makeInitialEstimate()),

      initial_covariance_(makeInitialCovariance()),

      // ========================================================
      // COVARIANCE MATRICES
      // ========================================================

      Q_(kf::StateMatrix::Zero()),

      R_gps_(2, 2),

      R_lidar_(2, 2),

      // ========================================================
      // MODELS
      // ========================================================

      vehicle_model_(vehicle_parameters_),

      gps_model_(),

      lidar_model_(landmark_),

      // ========================================================
      // SENSOR SIMULATORS
      // ========================================================

      gps_sensor_(gps_std_x_, gps_std_y_),

      lidar_sensor_(landmark_, lidar_range_std_, lidar_bearing_std_),

      // ========================================================
      // FILTER
      // ========================================================

      ekf_(initial_estimate_, initial_covariance_) {
  // ========================================================
  // PROCESS NOISE
  //
  // State:
  //
  // x = [ X, Y, v, psi ]
  //
  // Q =
  // [ qX    0     0      0    ]
  // [  0   qY     0      0    ]
  // [  0    0    qV      0    ]
  // [  0    0     0    qPsi   ]
  // ========================================================

  Q_.setZero();

  Q_(0, 0) = q_x_;

  Q_(1, 1) = q_y_;

  Q_(2, 2) = q_v_;

  Q_(3, 3) = q_psi_;

  // ========================================================
  // GPS MEASUREMENT COVARIANCE
  //
  // z_gps = [ X, Y ]
  //
  // Rgps =
  //
  // [ sigma_x^2      0      ]
  // [     0      sigma_y^2  ]
  // ========================================================

  R_gps_.setZero();

  R_gps_(0, 0) = gps_std_x_ * gps_std_x_;

  R_gps_(1, 1) = gps_std_y_ * gps_std_y_;

  // ========================================================
  // LIDAR MEASUREMENT COVARIANCE
  //
  // z_lidar = [ range, bearing ]
  //
  // Rlidar =
  //
  // [ sigma_range^2        0         ]
  // [      0        sigma_bearing^2  ]
  // ========================================================

  R_lidar_.setZero();

  R_lidar_(0, 0) = lidar_range_std_ * lidar_range_std_;

  R_lidar_(1, 1) = lidar_bearing_std_ * lidar_bearing_std_;

  // ========================================================
  // SENSOR SCHEDULING
  //
  // GPS   = 5 Hz  -> 0.20 s
  // LiDAR = 20 Hz -> 0.05 s
  //
  // First measurements occur at their first actual periods.
  // ========================================================

  next_gps_time_ = gps_period_;

  next_lidar_time_ = lidar_period_;
}

kf::StateVector kf::EKFSimulation::makeInitialTrueState() {
  kf::StateVector state;

  // [ X, Y, v, psi ]

  state << 0.0, 0.0, 10.0, 0.1;

  return state;
}

kf::StateVector kf::EKFSimulation::makeInitialEstimate() {
  kf::StateVector state;

  // Intentionally offset from ground truth

  state << 0.5, -0.3, 9.5, 0.15;

  return state;
}

kf::StateMatrix kf::EKFSimulation::makeInitialCovariance() {
  kf::StateMatrix covariance = kf::StateMatrix::Zero();

  covariance(0, 0) = 1.0;

  covariance(1, 1) = 1.0;

  covariance(2, 2) = 0.5;

  covariance(3, 3) = 0.2;

  return covariance;
}

void kf::EKFSimulation::reset() {
  // ========================================================
  // Reset ground truth
  // ========================================================

  true_state_ = initial_true_state_;

  // ========================================================
  // Reset sensors
  //
  // Reconstructing them also resets their random generators,
  // so repeated runs remain deterministic.
  // ========================================================

  gps_sensor_ = kf::GPSSensor(gps_std_x_, gps_std_y_);

  lidar_sensor_ =
      kf::LidarSensor(landmark_, lidar_range_std_, lidar_bearing_std_);

  // ========================================================
  // Reset EKF
  // ========================================================

  ekf_ = kf::ExtendedKalmanFilter(initial_estimate_, initial_covariance_);

  // ========================================================
  // Reset simulation clock
  // ========================================================

  simulation_time_ = 0.0;

  step_ = 0;

  // ========================================================
  // Reset asynchronous sensor schedules
  //
  // GPS:
  //   first measurement at 0.20 s
  //
  // LiDAR:
  //   first measurement at 0.05 s
  // ========================================================

  next_gps_time_ = gps_period_;

  next_lidar_time_ = lidar_period_;
}

// ============================================================
// SENSOR ENABLE / DISABLE
// ============================================================

void kf::EKFSimulation::setGpsEnabled(bool enabled) { gps_enabled_ = enabled; }

void kf::EKFSimulation::setLidarEnabled(bool enabled) {
  lidar_enabled_ = enabled;
}

bool kf::EKFSimulation::gpsEnabled() const { return gps_enabled_; }

bool kf::EKFSimulation::lidarEnabled() const { return lidar_enabled_; }

// ============================================================
// SENSOR / PREDICTION RATES
// ============================================================

kf::Scalar kf::EKFSimulation::predictionRateHz() const { return 1.0 / dt_; }

kf::Scalar kf::EKFSimulation::gpsRateHz() const { return gps_rate_hz_; }

kf::Scalar kf::EKFSimulation::lidarRateHz() const { return lidar_rate_hz_; }

// ============================================================
// GPS INFORMATION
// ============================================================

kf::Scalar kf::EKFSimulation::gpsStdX() const { return gps_std_x_; }

kf::Scalar kf::EKFSimulation::gpsStdY() const { return gps_std_y_; }

// ============================================================
// LIDAR INFORMATION
// ============================================================

kf::Scalar kf::EKFSimulation::lidarRangeStd() const { return lidar_range_std_; }

kf::Scalar kf::EKFSimulation::lidarBearingStd() const {
  return lidar_bearing_std_;
}

const kf::Landmark &kf::EKFSimulation::landmark() const { return landmark_; }

// ============================================================
// PROCESS NOISE INFORMATION
// ============================================================

kf::Scalar kf::EKFSimulation::qX() const { return q_x_; }

kf::Scalar kf::EKFSimulation::qY() const { return q_y_; }

kf::Scalar kf::EKFSimulation::qV() const { return q_v_; }

kf::Scalar kf::EKFSimulation::qPsi() const { return q_psi_; }

bool kf::EKFSimulation::step(kf::EstimatorSnapshot &snapshot) {
  if (step_ >= max_steps_) {
    return false;
  }

  // ========================================================
  // 0. CLEAR PER-STEP SENSOR STATUS / DIAGNOSTICS
  // ========================================================

  snapshot.gps_enabled = gps_enabled_;

  snapshot.lidar_enabled = lidar_enabled_;

  snapshot.gps_updated = false;

  snapshot.lidar_updated = false;

  snapshot.gps_measurement.resize(0);
  snapshot.gps_innovation.resize(0);
  snapshot.gps_kalman_gain.resize(4, 0);

  snapshot.lidar_measurement.resize(0);
  snapshot.lidar_innovation.resize(0);
  snapshot.lidar_kalman_gain.resize(4, 0);

  // Legacy viewer compatibility
  snapshot.innovation.resize(0);
  snapshot.kalman_gain.resize(4, 0);

  // ========================================================
  // 1. ADVANCE SIMULATION TIME
  // ========================================================

  simulation_time_ += dt_;

  // ========================================================
  // 2. PROPAGATE GROUND TRUTH
  // ========================================================

  true_state_ = vehicle_model_.propagate(true_state_, control_input_, dt_);

  // ========================================================
  // 3. EKF PREDICTION
  //
  // Runs every base step.
  //
  // dt = 0.01 s  -> 100 Hz
  // ========================================================

  const kf::StateVector current_estimate = ekf_.state();

  const kf::StateMatrix F =
      vehicle_model_.jacobian(current_estimate, control_input_, dt_);

  const kf::StateVector predicted_state =
      vehicle_model_.propagate(current_estimate, control_input_, dt_);

  ekf_.predict(predicted_state, F, Q_);

  // Preserve PRIOR before any sensor update

  const kf::StateMatrix predicted_covariance = ekf_.covariance();

  // ========================================================
  // 4. GPS UPDATE
  //
  // GPS = 5 Hz
  //
  // period = 0.20 s
  // ========================================================

  if (gps_enabled_ && simulation_time_ + 1e-12 >= next_gps_time_) {
    // ----------------------------------------------------
    // Actual noisy GPS measurement
    // ----------------------------------------------------

    const kf::MeasurementVector gps_z = gps_sensor_.measure(true_state_);

    // ----------------------------------------------------
    // GPS linear measurement model
    //
    // z = Hx
    // ----------------------------------------------------

    const kf::MeasurementMatrix H_gps = gps_model_.measurementMatrix();

    const kf::MeasurementVector predicted_gps = H_gps * ekf_.state();

    const kf::MeasurementVector gps_innovation = gps_z - predicted_gps;

    // ----------------------------------------------------
    // EKF correction using GPS
    // ----------------------------------------------------

    ekf_.update(gps_innovation, H_gps, R_gps_);

    // ----------------------------------------------------
    // Store GPS diagnostics
    // ----------------------------------------------------

    snapshot.gps_updated = true;

    snapshot.gps_measurement = gps_z;

    snapshot.gps_innovation = ekf_.innovation();

    snapshot.gps_kalman_gain = ekf_.kalmanGain();

    // Legacy viewer fields
    snapshot.innovation = ekf_.innovation();

    snapshot.kalman_gain = ekf_.kalmanGain();

    // ----------------------------------------------------
    // Schedule next GPS sample
    // ----------------------------------------------------

    next_gps_time_ += gps_period_;
  }

  // ========================================================
  // 5. LIDAR UPDATE
  //
  // LiDAR = 20 Hz
  //
  // period = 0.05 s
  //
  // IMPORTANT:
  //
  // If GPS updated above, then ekf_.state() already contains
  // the GPS-corrected state.
  //
  // Therefore LiDAR linearization uses the newest state.
  // ========================================================

  if (lidar_enabled_ && simulation_time_ + 1e-12 >= next_lidar_time_) {
    // ----------------------------------------------------
    // Actual LiDAR measurement
    // ----------------------------------------------------

    const kf::MeasurementVector lidar_z = lidar_sensor_.measure(true_state_);

    // ----------------------------------------------------
    // Linearize LiDAR around current EKF state
    // ----------------------------------------------------

    const kf::StateVector lidar_linearization_state = ekf_.state();

    const kf::MeasurementVector lidar_innovation =
        lidar_model_.innovation(lidar_z, lidar_linearization_state);

    const kf::MeasurementMatrix H_lidar =
        lidar_model_.jacobian(lidar_linearization_state);

    // ----------------------------------------------------
    // EKF correction using LiDAR
    // ----------------------------------------------------

    ekf_.update(lidar_innovation, H_lidar, R_lidar_);

    // ----------------------------------------------------
    // Store LiDAR diagnostics
    // ----------------------------------------------------

    snapshot.lidar_updated = true;

    snapshot.lidar_measurement = lidar_z;

    snapshot.lidar_innovation = ekf_.innovation();

    snapshot.lidar_kalman_gain = ekf_.kalmanGain();

    // Legacy viewer fields
    //
    // If GPS and LiDAR both update at this step,
    // these now contain LiDAR diagnostics because
    // LiDAR was the last update.
    snapshot.innovation = ekf_.innovation();

    snapshot.kalman_gain = ekf_.kalmanGain();

    // ----------------------------------------------------
    // Schedule next LiDAR sample
    // ----------------------------------------------------

    next_lidar_time_ += lidar_period_;
  }

  // ========================================================
  // 6. FILL COMMON SNAPSHOT
  // ========================================================

  snapshot.time = simulation_time_;

  snapshot.true_state = true_state_;

  snapshot.predicted_state = predicted_state;

  snapshot.predicted_covariance = predicted_covariance;

  snapshot.estimated_state = ekf_.state();

  snapshot.estimated_covariance = ekf_.covariance();

  // ========================================================
  // 7. ADVANCE STEP COUNTER
  // ========================================================

  ++step_;

  return true;
}