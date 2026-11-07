#pragma once

#include <functional>
#include <vector>

#include "kf/core/EstimatorSnapshot.hpp"
#include "kf/core/SimulationMode.hpp"
#include "kf/core/Types.hpp"

struct GLFWwindow;

namespace kf {

class RealTimeViewer {
public:
  // ------------------------------------------------------------
  // One simulation step
  // ------------------------------------------------------------

  using StepCallback = std::function<bool(EstimatorSnapshot &)>;

  // ------------------------------------------------------------
  // Reset selected simulation
  // ------------------------------------------------------------

  using ResetCallback = std::function<void()>;
  using SimulationDtCallback = std::function<Scalar()>;

  RealTimeViewer();

  ~RealTimeViewer();

  bool initialize();

  void run(const std::vector<EstimatorSnapshot> &history);

  void runLive(std::vector<EstimatorSnapshot> &history,

               const StepCallback &step_callback,

               const ResetCallback &reset_callback,

               const SimulationDtCallback &simulation_dt_callback,

               SimulationMode &simulation_mode,

               // LKF
               Scalar lkf_qx, Scalar lkf_qy, Scalar lkf_rx, Scalar lkf_ry,

               // EKF process noise
               Scalar ekf_qx, Scalar ekf_qy, Scalar ekf_qv, Scalar ekf_qpsi,
               // EKF sensor rates
               Scalar ekf_gps_rate_hz, Scalar ekf_lidar_rate_hz,

               // LiDAR
               Scalar lidar_range_std, Scalar lidar_bearing_std,

               // Landmark
               Scalar landmark_x, Scalar landmark_y);

  void shutdown();

private:
  GLFWwindow *window_;

  bool initialized_;
};

} // namespace kf