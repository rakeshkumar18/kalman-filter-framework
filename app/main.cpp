#include <iostream>
#include <vector>

#include "kf/core/EstimatorSnapshot.hpp"
#include "kf/core/SimulationMode.hpp"
#include "kf/core/Types.hpp"

#include "kf/simulation/VehicleSimulator.hpp"

#include "kf/sensors/GPSSensor.hpp"
#include "kf/sensors/LidarSensor.hpp"

#include "kf/models/ConstantVelocityModel.hpp"
#include "kf/models/GPSMeasurementModel.hpp"
#include "kf/models/LidarMeasurementModel.hpp"
#include "kf/models/NonlinearVehicleModel.hpp"

#include "kf/filters/ExtendedKalmanFilter.hpp"
#include "kf/filters/LinearKalmanFilter.hpp"

#include "kf/simulation/EKFSimulation.hpp"
#include "kf/visualization/RealTimeViewer.hpp"

namespace {

// ============================================================
// LKF SIMULATION
// ============================================================

class LKFSimulation {
public:
  explicit LKFSimulation(kf::Scalar dt)
      : dt_(dt),

        initial_true_state_(makeInitialTrueState()),

        initial_estimate_(initial_true_state_),

        initial_covariance_(kf::StateMatrix::Identity()),

        simulator_(initial_true_state_),

        gps_(gps_std_x_, gps_std_y_),

        filter_(initial_estimate_, initial_covariance_),

        H_(gps_model_.measurementMatrix()),

        R_(2, 2) {
    acc_cov_.setZero();

    acc_cov_(0, 0) = qx_;

    acc_cov_(1, 1) = qy_;

    R_ << rx_, 0.0, 0.0, ry_;
  }

  bool step(kf::EstimatorSnapshot &snapshot) {
    if (step_ >= max_steps_) {
      return false;
    }

    // ====================================================
    // Ground truth
    // ====================================================

    simulator_.step(dt_, ax_, ay_);

    const kf::StateVector true_state = simulator_.state();

    // ====================================================
    // GPS measurement
    // ====================================================

    const kf::MeasurementVector z = gps_.measure(true_state);

    // ====================================================
    // LKF model
    // ====================================================

    const kf::StateMatrix F = motion_model_.stateTransition(dt_);

    const kf::StateMatrix Q = motion_model_.processNoise(dt_, acc_cov_);

    // ====================================================
    // Prediction
    // ====================================================

    filter_.predict(F, Q);

    const kf::StateVector predicted_state = filter_.state();

    const kf::StateMatrix predicted_covariance = filter_.covariance();

    // ====================================================
    // Update
    // ====================================================

    filter_.update(z, H_, R_);

    // ====================================================
    // Snapshot
    // ====================================================

    snapshot.time = static_cast<kf::Scalar>(step_ + 1) * dt_;

    snapshot.true_state = true_state;

    snapshot.gps_measurement = z;

    // No LiDAR measurement in LKF mode.
    snapshot.lidar_measurement.resize(0);

    snapshot.predicted_state = predicted_state;

    snapshot.predicted_covariance = predicted_covariance;

    snapshot.estimated_state = filter_.state();

    snapshot.estimated_covariance = filter_.covariance();

    // snapshot.innovation = filter_.innovation();

    // snapshot.kalman_gain = filter_.kalmanGain();

    ++step_;

    return true;
  }

  void reset() {
    simulator_ = kf::VehicleSimulator(initial_true_state_);

    gps_ = kf::GPSSensor(gps_std_x_, gps_std_y_);

    filter_ = kf::LinearKalmanFilter(initial_estimate_, initial_covariance_);

    step_ = 0;
  }

  kf::Scalar qx() const { return qx_; }

  kf::Scalar qy() const { return qy_; }

  kf::Scalar rx() const { return rx_; }

  kf::Scalar ry() const { return ry_; }

private:
  static kf::StateVector makeInitialTrueState() {
    kf::StateVector state;

    state << 0.0, 0.0, 5.0, 1.0;

    return state;
  }

  kf::Scalar dt_;

  kf::Scalar ax_ = 0.2;

  kf::Scalar ay_ = 0.0;

  // Process acceleration variances
  kf::Scalar qx_ = 0.2;

  kf::Scalar qy_ = 0.05;

  // GPS measurement variances
  kf::Scalar rx_ = 1.0;

  kf::Scalar ry_ = 2.25;

  kf::Scalar gps_std_x_ = 1.0;

  kf::Scalar gps_std_y_ = 1.5;

  int step_ = 0;

  int max_steps_ = 500;

  kf::StateVector initial_true_state_;

  kf::StateVector initial_estimate_;

  kf::StateMatrix initial_covariance_;

  kf::VehicleSimulator simulator_;

  kf::GPSSensor gps_;

  kf::ConstantVelocityModel motion_model_;

  kf::GPSMeasurementModel gps_model_;

  kf::LinearKalmanFilter filter_;

  kf::MeasurementMatrix H_;

  kf::AccelerationCovariance acc_cov_;

  kf::MeasurementCovariance R_;
};

} // anonymous namespace

// ============================================================
// MAIN
// ============================================================

int main() {

  // ========================================================
  // SIMULATION SAMPLE TIMES
  // ========================================================

  constexpr kf::Scalar lkf_dt = 0.1;

  constexpr kf::Scalar ekf_dt = 0.01;

  // ========================================================
  // SIMULATIONS
  // ========================================================

  LKFSimulation lkf_simulation(lkf_dt);

  kf::EKFSimulation ekf_simulation(ekf_dt);

  // ========================================================
  // INITIAL MODE
  // ========================================================

  kf::SimulationMode simulation_mode = kf::SimulationMode::LKF;

  // ========================================================
  // SHARED VIEWER HISTORY
  // ========================================================

  std::vector<kf::EstimatorSnapshot> history;

  history.reserve(5000);

  // ========================================================
  // Unified simulation step
  // ========================================================

  auto step_callback = [&](kf::EstimatorSnapshot &snapshot) -> bool {
    switch (simulation_mode) {
    case kf::SimulationMode::LKF: {
      return lkf_simulation.step(snapshot);
    }

    case kf::SimulationMode::EKF: {
      return ekf_simulation.step(snapshot);
    }

    case kf::SimulationMode::UKF: {
      // UKF is intentionally not implemented yet.
      return false;
    }
    }

    return false;
  };

  // ========================================================
  // Unified reset
  // ========================================================

  auto reset_callback = [&]() {
    switch (simulation_mode) {
    case kf::SimulationMode::LKF: {
      lkf_simulation.reset();
      break;
    }

    case kf::SimulationMode::EKF: {
      ekf_simulation.reset();
      break;
    }

    case kf::SimulationMode::UKF: {
      // Nothing to reset yet.
      break;
    }
    }
  };

  auto simulation_dt_callback = [&]() -> kf::Scalar {
    switch (simulation_mode) {
    case kf::SimulationMode::LKF: {
      return lkf_dt;
    }

    case kf::SimulationMode::EKF: {
      return ekf_dt;
    }

    case kf::SimulationMode::UKF: {
      return ekf_dt;
    }
    }

    return ekf_dt;
  };

  // ========================================================
  // Viewer
  // ========================================================

  kf::RealTimeViewer viewer;

  if (!viewer.initialize()) {
    std::cerr << "Failed to initialize viewer\n";

    return 1;
  }

  viewer.runLive(history, step_callback, reset_callback, simulation_dt_callback,
                 simulation_mode,

                 // LKF parameters
                 lkf_simulation.qx(), lkf_simulation.qy(), lkf_simulation.rx(),
                 lkf_simulation.ry(),

                 // EKF process parameters
                 ekf_simulation.qX(), ekf_simulation.qY(), ekf_simulation.qV(),
                 ekf_simulation.qPsi(),

                 ekf_simulation.gpsRateHz(), ekf_simulation.lidarRateHz(),

                 // LiDAR
                 ekf_simulation.lidarRangeStd(),
                 ekf_simulation.lidarBearingStd(),

                 // Landmark
                 ekf_simulation.landmark().x, ekf_simulation.landmark().y);

  return 0;
}