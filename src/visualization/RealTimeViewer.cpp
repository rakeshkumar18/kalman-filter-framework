#include "kf/visualization/RealTimeViewer.hpp"
#include "kf/core/SimulationMode.hpp"

#include <Eigen/Eigenvalues>
#include <GLFW/glfw3.h>
#include <cmath>

#include "imgui.h"
#include "implot.h"

#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

using ResetCallback = std::function<void()>;

namespace {

void buildCovarianceEllipse(const kf::StateVector &center,
                            const kf::StateMatrix &cov,
                            std::vector<double> &ellipse_x,
                            std::vector<double> &ellipse_y,
                            double sigma_scale = 2.0) {
  Eigen::Matrix2d position_cov = cov.block<2, 2>(0, 0);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(position_cov);

  if (solver.info() != Eigen::Success) {
    return;
  }

  Eigen::Vector2d eigenvalues = solver.eigenvalues();
  Eigen::Matrix2d eigenvectors = solver.eigenvectors();

  // Numerical protection

  eigenvalues(0) = std::max(eigenvalues(0), 0.0);
  eigenvalues(1) = std::max(eigenvalues(1), 0.0);

  const int number_of_points = 80;

  ellipse_x.clear();
  ellipse_y.clear();

  ellipse_x.reserve(number_of_points + 1);
  ellipse_y.reserve(number_of_points + 1);

  for (int i = 0; i <= number_of_points; ++i) {
    double theta = 2.0 * M_PI * static_cast<double>(i) /
                   static_cast<double>(number_of_points);

    Eigen::Vector2d local_point;

    local_point(0) = sigma_scale * std::sqrt(eigenvalues(0)) * std::cos(theta);

    local_point(1) = sigma_scale * std::sqrt(eigenvalues(1)) * std::sin(theta);

    Eigen::Vector2d rotated_point = eigenvectors * local_point;

    ellipse_x.push_back(center(0) + rotated_point(0));

    ellipse_y.push_back(center(1) + rotated_point(1));
  }
}

} // namespace

kf::RealTimeViewer::RealTimeViewer() : window_(nullptr), initialized_(false) {}

kf::RealTimeViewer::~RealTimeViewer() { shutdown(); }

bool kf::RealTimeViewer::initialize() {
  // --------------------------------------------
  // 1. Initialize GLFW
  // --------------------------------------------
  if (!glfwInit()) {
    return false;
  }

#ifdef __APPLE__

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);

  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);

  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

#endif

  // --------------------------------------------
  // 2. Create window
  // --------------------------------------------
  window_ =
      glfwCreateWindow(1280, 720, "Kalman Filter Simulation", nullptr, nullptr);

  if (window_ == nullptr) {
    glfwTerminate();
    return false;
  }

  // --------------------------------------------
  // 3. OpenGL context
  // --------------------------------------------
  glfwMakeContextCurrent(window_);

  glfwSwapInterval(1);

  // --------------------------------------------
  // 4. Create ImGui / ImPlot contexts
  // --------------------------------------------
  IMGUI_CHECKVERSION();

  ImGui::CreateContext();

  ImPlot::CreateContext();

  ImGui::StyleColorsDark();

  // --------------------------------------------
  // 5. Initialize ImGui GLFW backend
  // --------------------------------------------
  if (!ImGui_ImplGlfw_InitForOpenGL(window_, true)) {
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window_);
    window_ = nullptr;

    glfwTerminate();

    return false;
  }

  // --------------------------------------------
  // 6. Initialize ImGui OpenGL backend
  // --------------------------------------------
  const char *glsl_version = "#version 150";

  if (!ImGui_ImplOpenGL3_Init(glsl_version)) {
    ImGui_ImplGlfw_Shutdown();

    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window_);
    window_ = nullptr;

    glfwTerminate();

    return false;
  }

  initialized_ = true;

  return true;
}

void kf::RealTimeViewer::shutdown() {
  if (!initialized_) {
    return;
  }

  // Reverse order of initialization
  ImGui_ImplOpenGL3_Shutdown();

  ImGui_ImplGlfw_Shutdown();

  ImPlot::DestroyContext();

  ImGui::DestroyContext();

  if (window_ != nullptr) {
    glfwDestroyWindow(window_);

    window_ = nullptr;
  }

  glfwTerminate();

  initialized_ = false;
}

void kf::RealTimeViewer::run(
    const std::vector<kf::EstimatorSnapshot> &history) {
  while (!glfwWindowShouldClose(window_)) {
    // --------------------------------------------
    // 1. Process keyboard / mouse / window events
    // --------------------------------------------
    glfwPollEvents();

    // --------------------------------------------
    // 2. Start new ImGui frame
    // --------------------------------------------
    ImGui_ImplOpenGL3_NewFrame();

    ImGui_ImplGlfw_NewFrame();

    ImGui::NewFrame();
    // --------------------------------------------
    // 3. Build GUI
    // --------------------------------------------

    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);

    ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize, ImGuiCond_Always);

    ImGui::Begin("Kalman Filter Simulation", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoCollapse);

    // --------------------------------------------------
    // Available space inside main window
    // --------------------------------------------------

    ImVec2 available = ImGui::GetContentRegionAvail();

    float left_width = available.x * 0.68f;

    float right_width = available.x - left_width;

    float top_height = available.y * 0.62f;

    float bottom_height = available.y - top_height;

    // --------------------------------------------------
    // Top-left: XY trajectory
    // --------------------------------------------------

    ImGui::BeginChild("XYTrajectoryPanel", ImVec2(left_width, top_height),
                      true);

    ImGui::Text("XY Trajectory");

    ImGui::Separator();

    // --------------------------------------------------
    // Prepare trajectory data
    // --------------------------------------------------

    std::vector<double> true_x;
    std::vector<double> true_y;

    std::vector<double> gps_x;
    std::vector<double> gps_y;

    std::vector<double> predicted_x;
    std::vector<double> predicted_y;

    std::vector<double> estimate_x;
    std::vector<double> estimate_y;

    std::vector<double> prior_ellipse_x;
    std::vector<double> prior_ellipse_y;

    std::vector<double> posterior_ellipse_x;
    std::vector<double> posterior_ellipse_y;

    std::vector<double> time_data;

    std::vector<double> error_x;
    std::vector<double> error_y;

    true_x.reserve(history.size());
    true_y.reserve(history.size());

    gps_x.reserve(history.size());
    gps_y.reserve(history.size());

    predicted_x.reserve(history.size());
    predicted_y.reserve(history.size());

    estimate_x.reserve(history.size());
    estimate_y.reserve(history.size());

    time_data.reserve(history.size());

    error_x.reserve(history.size());
    error_y.reserve(history.size());

    for (const auto &snapshot : history) {
      true_x.push_back(snapshot.true_state(0));

      true_y.push_back(snapshot.true_state(1));

      gps_x.push_back(snapshot.gps_measurement(0));

      gps_y.push_back(snapshot.gps_measurement(1));

      predicted_x.push_back(snapshot.predicted_state(0));

      predicted_y.push_back(snapshot.predicted_state(1));

      estimate_x.push_back(snapshot.estimated_state(0));

      estimate_y.push_back(snapshot.estimated_state(1));

      time_data.push_back(snapshot.time);

      error_x.push_back(snapshot.estimated_state(0) - snapshot.true_state(0));

      error_y.push_back(snapshot.estimated_state(1) - snapshot.true_state(1));
    }

    // --------------------------------------------------
    // Plot
    // --------------------------------------------------

    if (ImPlot::BeginPlot("##XYTrajectory", ImVec2(-1, -1))) {
      ImPlot::SetupAxes("X position [m]", "Y position [m]");

      ImPlot::PlotLine("Truth", true_x.data(), true_y.data(),
                       static_cast<int>(true_x.size()));

      ImPlot::PlotScatter("GPS", gps_x.data(), gps_y.data(),
                          static_cast<int>(gps_x.size()));
      ImPlot::PlotLine("KF Prediction", predicted_x.data(), predicted_y.data(),
                       static_cast<int>(predicted_x.size()));

      ImPlot::PlotLine("KF Estimate", estimate_x.data(), estimate_y.data(),
                       static_cast<int>(estimate_x.size()));

      ImPlot::EndPlot();
    }

    ImGui::EndChild();

    // Same row
    ImGui::SameLine();

    // --------------------------------------------------
    // Top-right: Filter parameters
    // --------------------------------------------------

    ImGui::BeginChild("FilterParametersPanel", ImVec2(right_width, top_height),
                      true);

    ImGui::Text("Filter Parameters");

    ImGui::Separator();

    ImGui::Text("Qx");
    ImGui::Text("Qy");
    ImGui::Text("Rx");
    ImGui::Text("Ry");

    ImGui::Spacing();

    ImGui::Text("GPS available");
    ImGui::Text("Add GPS bias");
    ImGui::Text("Inject outlier");

    ImGui::EndChild();

    // ==================================================
    // BOTTOM-LEFT
    // POSITION ERROR
    // ==================================================

    ImGui::BeginChild("PositionErrorPanel", ImVec2(left_width, bottom_height),
                      true);

    ImGui::Text("Position Error vs Time");
    ImGui::Separator();

    ImVec2 error_plot_size = ImGui::GetContentRegionAvail();

    if (error_plot_size.x > 10.0f && error_plot_size.y > 10.0f) {
      if (ImPlot::BeginPlot("##PositionError", error_plot_size)) {
        ImPlot::SetupAxis(ImAxis_X1, "Time [s]", ImPlotAxisFlags_AutoFit);

        ImPlot::SetupAxis(ImAxis_Y1, "Position error [m]",
                          ImPlotAxisFlags_AutoFit);

        if (!time_data.empty() && time_data.size() == error_x.size() &&
            time_data.size() == error_y.size()) {
          ImPlot::PlotLine("ex", time_data.data(), error_x.data(),
                           static_cast<int>(time_data.size()));

          ImPlot::PlotLine("ey", time_data.data(), error_y.data(),
                           static_cast<int>(time_data.size()));
        }

        ImPlot::EndPlot();
      }
    }

    ImGui::EndChild();

    // --------------------------------------------------
    // Bottom-right: Diagnostics
    // --------------------------------------------------

    ImGui::BeginChild("DiagnosticsPanel", ImVec2(right_width, bottom_height),
                      true);

    ImGui::Text("Filter Diagnostics");

    ImGui::Separator();

    ImGui::Text("Innovation");
    ImGui::Text("Kalman gain");
    ImGui::Text("Trace(P)");
    ImGui::Text("Current state");
    ImGui::Text("Execution time");

    ImGui::EndChild();

    ImGui::End();
    ImGui::Render();

    // --------------------------------------------
    // 5. Get actual framebuffer size
    // --------------------------------------------
    int display_width;
    int display_height;

    glfwGetFramebufferSize(window_, &display_width, &display_height);

    glViewport(0, 0, display_width, display_height);

    // --------------------------------------------
    // 6. Clear background
    // --------------------------------------------
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

    glClear(GL_COLOR_BUFFER_BIT);

    // --------------------------------------------
    // 7. Render ImGui
    // --------------------------------------------
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // --------------------------------------------
    // 8. Display frame
    // --------------------------------------------
    glfwSwapBuffers(window_);
  }
}

void kf::RealTimeViewer::runLive(
    std::vector<kf::EstimatorSnapshot> &history,

    const kf::RealTimeViewer::StepCallback &step_callback,

    const kf::RealTimeViewer::ResetCallback &reset_callback,

    const kf::RealTimeViewer::SimulationDtCallback &simulation_dt_callback,

    kf::SimulationMode &simulation_mode,

    // LKF
    kf::Scalar lkf_qx, kf::Scalar lkf_qy, kf::Scalar lkf_rx, kf::Scalar lkf_ry,

    // EKF
    kf::Scalar ekf_qx, kf::Scalar ekf_qy, kf::Scalar ekf_qv,
    kf::Scalar ekf_qpsi,

    kf::Scalar ekf_gps_rate_hz, kf::Scalar ekf_lidar_rate_hz,

    // LiDAR
    kf::Scalar lidar_range_std, kf::Scalar lidar_bearing_std,

    // Landmark
    kf::Scalar landmark_x, kf::Scalar landmark_y) {
  // ============================================================
  // SIMULATION STATE
  // ============================================================

  double last_step_time = glfwGetTime();

  bool simulation_active = (simulation_mode != kf::SimulationMode::UKF);

  bool simulation_running = false;

  // ============================================================
  // MAIN APPLICATION LOOP
  // ============================================================

  while (!glfwWindowShouldClose(window_)) {
    // --------------------------------------------------------
    // GLFW events
    // --------------------------------------------------------

    glfwPollEvents();

    // --------------------------------------------------------
    // Advance simulation
    // --------------------------------------------------------

    const double current_time = glfwGetTime();
    const kf::Scalar sim_dt = simulation_dt_callback();

    int catchup_steps = 0;

    constexpr int max_catchup_steps = 20;

    while (simulation_active && simulation_running &&
           (current_time - last_step_time) >= sim_dt &&
           catchup_steps < max_catchup_steps) {
      kf::EstimatorSnapshot snapshot;

      if (step_callback(snapshot)) {
        history.push_back(snapshot);
      } else {
        simulation_active = false;

        simulation_running = false;

        break;
      }

      last_step_time += sim_dt;

      ++catchup_steps;
    }

    // ========================================================
    // START IMGUI FRAME
    // ========================================================

    ImGui_ImplOpenGL3_NewFrame();

    ImGui_ImplGlfw_NewFrame();

    ImGui::NewFrame();

    // ========================================================
    // MAIN WINDOW
    // ========================================================

    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);

    ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize, ImGuiCond_Always);

    ImGui::Begin("Kalman Filter Simulation", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoCollapse);

    // ========================================================
    // ESTIMATOR SELECTION
    // ========================================================

    ImGui::Text("Estimator");

    ImGui::SameLine();

    int selected_mode = static_cast<int>(simulation_mode);

    ImGui::RadioButton("LKF", &selected_mode,
                       static_cast<int>(kf::SimulationMode::LKF));

    ImGui::SameLine();

    ImGui::RadioButton("EKF", &selected_mode,
                       static_cast<int>(kf::SimulationMode::EKF));

    ImGui::SameLine();

    ImGui::RadioButton("UKF", &selected_mode,
                       static_cast<int>(kf::SimulationMode::UKF));

    const kf::SimulationMode new_mode =
        static_cast<kf::SimulationMode>(selected_mode);

    // ========================================================
    // HANDLE MODE CHANGE
    // ========================================================

    if (new_mode != simulation_mode) {
      simulation_running = false;

      simulation_mode = new_mode;

      // Never mix snapshots from different filters.
      history.clear();

      // reset_callback uses simulation_mode internally,
      // therefore it resets the newly selected simulation.
      reset_callback();

      simulation_active = (simulation_mode != kf::SimulationMode::UKF);

      last_step_time = glfwGetTime();
    }

    ImGui::Separator();

    // ========================================================
    // UKF PLACEHOLDER
    // ========================================================

    if (simulation_mode == kf::SimulationMode::UKF) {
      ImGui::Spacing();
      ImGui::Spacing();

      ImGui::Text("Unscented Kalman Filter");

      ImGui::Spacing();

      ImGui::TextDisabled("Implementing soon");

      ImGui::Spacing();

      ImGui::TextDisabled("Select LKF or EKF above to run a simulation.");

      ImGui::End();

      // ----------------------------------------------------
      // Render UKF placeholder frame
      // ----------------------------------------------------

      ImGui::Render();

      int display_width = 0;

      int display_height = 0;

      glfwGetFramebufferSize(window_, &display_width, &display_height);

      glViewport(0, 0, display_width, display_height);

      glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

      glClear(GL_COLOR_BUFFER_BIT);

      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

      glfwSwapBuffers(window_);

      continue;
    }

    // ========================================================
    // DASHBOARD GEOMETRY
    // ========================================================

    const ImVec2 available = ImGui::GetContentRegionAvail();

    const float horizontal_spacing = ImGui::GetStyle().ItemSpacing.x;

    const float vertical_spacing = ImGui::GetStyle().ItemSpacing.y;

    const float left_width = available.x * 0.68f;

    const float right_width = available.x - left_width - horizontal_spacing;

    const float top_height = available.y * 0.62f;

    const float bottom_height = available.y - top_height - vertical_spacing;

    // ========================================================
    // PREPARE PLOT DATA
    // ========================================================

    std::vector<double> true_x;
    std::vector<double> true_y;

    std::vector<double> gps_x;
    std::vector<double> gps_y;

    std::vector<double> predicted_x;
    std::vector<double> predicted_y;

    std::vector<double> estimate_x;
    std::vector<double> estimate_y;

    std::vector<double> time_data;

    std::vector<double> error_x;
    std::vector<double> error_y;

    const std::size_t sample_count = history.size();

    true_x.reserve(sample_count);

    true_y.reserve(sample_count);

    gps_x.reserve(sample_count);

    gps_y.reserve(sample_count);

    predicted_x.reserve(sample_count);

    predicted_y.reserve(sample_count);

    estimate_x.reserve(sample_count);

    estimate_y.reserve(sample_count);

    time_data.reserve(sample_count);

    error_x.reserve(sample_count);

    error_y.reserve(sample_count);

    for (const auto &snapshot : history) {
      // ----------------------------------------------------
      // Truth
      // ----------------------------------------------------

      true_x.push_back(snapshot.true_state(0));

      true_y.push_back(snapshot.true_state(1));

      // ----------------------------------------------------
      // GPS only exists for LKF mode
      // ----------------------------------------------------

      if (simulation_mode == kf::SimulationMode::LKF) {
        if (snapshot.gps_measurement.size() >= 2) {
          gps_x.push_back(snapshot.gps_measurement(0));

          gps_y.push_back(snapshot.gps_measurement(1));
        }
      }

      // ----------------------------------------------------
      // Prediction
      // ----------------------------------------------------

      predicted_x.push_back(snapshot.predicted_state(0));

      predicted_y.push_back(snapshot.predicted_state(1));

      // ----------------------------------------------------
      // Posterior estimate
      // ----------------------------------------------------

      estimate_x.push_back(snapshot.estimated_state(0));

      estimate_y.push_back(snapshot.estimated_state(1));

      // ----------------------------------------------------
      // Time
      // ----------------------------------------------------

      time_data.push_back(snapshot.time);

      // ----------------------------------------------------
      // Position errors
      // ----------------------------------------------------

      error_x.push_back(snapshot.estimated_state(0) - snapshot.true_state(0));

      error_y.push_back(snapshot.estimated_state(1) - snapshot.true_state(1));
    }

    // ========================================================
    // COVARIANCE ELLIPSES
    // ========================================================

    std::vector<double> prior_ellipse_x;
    std::vector<double> prior_ellipse_y;

    std::vector<double> posterior_ellipse_x;
    std::vector<double> posterior_ellipse_y;

    if (!history.empty()) {
      const auto &latest = history.back();

      buildCovarianceEllipse(latest.predicted_state,
                             latest.predicted_covariance, prior_ellipse_x,
                             prior_ellipse_y);

      buildCovarianceEllipse(latest.estimated_state,
                             latest.estimated_covariance, posterior_ellipse_x,
                             posterior_ellipse_y);
    }

    // ========================================================
    // RMSE
    // ========================================================

    double rmse_x = 0.0;

    double rmse_y = 0.0;

    double rmse_position = 0.0;

    if (!history.empty()) {
      double sum_squared_error_x = 0.0;

      double sum_squared_error_y = 0.0;

      for (std::size_t i = 0; i < error_x.size(); ++i) {
        sum_squared_error_x += error_x[i] * error_x[i];

        sum_squared_error_y += error_y[i] * error_y[i];
      }

      const double n = static_cast<double>(error_x.size());

      if (n > 0.0) {
        rmse_x = std::sqrt(sum_squared_error_x / n);

        rmse_y = std::sqrt(sum_squared_error_y / n);

        rmse_position =
            std::sqrt((sum_squared_error_x + sum_squared_error_y) / n);
      }
    }

    // ========================================================
    // TOP LEFT
    // XY TRAJECTORY
    // ========================================================

    ImGui::BeginChild("XYTrajectoryPanel", ImVec2(left_width, top_height),
                      true);

    if (simulation_mode == kf::SimulationMode::LKF) {
      ImGui::Text("LKF - XY Trajectory");
    } else {
      ImGui::Text("EKF - Nonlinear Vehicle Trajectory");
    }

    ImGui::Separator();

    if (ImPlot::BeginPlot("##XYTrajectory", ImVec2(-1, -1))) {
      ImPlot::SetupAxis(ImAxis_X1, "X position [m]", ImPlotAxisFlags_AutoFit);

      ImPlot::SetupAxis(ImAxis_Y1, "Y position [m]", ImPlotAxisFlags_AutoFit);

      if (!history.empty()) {
        // ------------------------------------------------
        // Truth
        // ------------------------------------------------

        ImPlot::PlotLine("Truth", true_x.data(), true_y.data(),
                         static_cast<int>(true_x.size()));

        // ------------------------------------------------
        // GPS only for LKF
        // ------------------------------------------------

        if (simulation_mode == kf::SimulationMode::LKF) {
          if (!gps_x.empty()) {
            ImPlot::PlotScatter("GPS", gps_x.data(), gps_y.data(),
                                static_cast<int>(gps_x.size()));
          }
        }

        // ------------------------------------------------
        // Prediction
        // ------------------------------------------------

        const char *prediction_label =
            (simulation_mode == kf::SimulationMode::LKF) ? "LKF Prediction"
                                                         : "EKF Prediction";

        ImPlot::PlotLine(prediction_label, predicted_x.data(),
                         predicted_y.data(),
                         static_cast<int>(predicted_x.size()));

        // ------------------------------------------------
        // Estimate
        // ------------------------------------------------

        const char *estimate_label =
            (simulation_mode == kf::SimulationMode::LKF) ? "LKF Estimate"
                                                         : "EKF Estimate";

        ImPlot::PlotLine(estimate_label, estimate_x.data(), estimate_y.data(),
                         static_cast<int>(estimate_x.size()));

        // ------------------------------------------------
        // Covariance ellipses
        // ------------------------------------------------

        if (!prior_ellipse_x.empty()) {
          ImPlot::PlotLine("Prior 2-sigma", prior_ellipse_x.data(),
                           prior_ellipse_y.data(),
                           static_cast<int>(prior_ellipse_x.size()));
        }

        if (!posterior_ellipse_x.empty()) {
          ImPlot::PlotLine("Posterior 2-sigma", posterior_ellipse_x.data(),
                           posterior_ellipse_y.data(),
                           static_cast<int>(posterior_ellipse_x.size()));
        }
      }

      // ----------------------------------------------------
      // LiDAR landmark exists for EKF even before Start
      // ----------------------------------------------------

      if (simulation_mode == kf::SimulationMode::EKF) {
        double landmark_plot_x = landmark_x;

        double landmark_plot_y = landmark_y;

        ImPlot::PlotScatter("LiDAR Landmark", &landmark_plot_x,
                            &landmark_plot_y, 1);
      }

      ImPlot::EndPlot();
    }

    ImGui::EndChild();

    // ========================================================
    // TOP RIGHT
    // PARAMETERS / CONTROLS
    // ========================================================

    ImGui::SameLine();

    ImGui::BeginChild("FilterParametersPanel", ImVec2(right_width, top_height),
                      true);

    ImGui::Text("Filter Parameters");

    ImGui::Separator();

    // --------------------------------------------------------
    // LKF parameters
    // --------------------------------------------------------

    if (simulation_mode == kf::SimulationMode::LKF) {
      ImGui::Text("Linear Kalman Filter");

      ImGui::Spacing();

      ImGui::Text("Process noise");

      ImGui::Text("Qx : %.4f", lkf_qx);

      ImGui::Text("Qy : %.4f", lkf_qy);

      ImGui::Spacing();

      ImGui::Text("GPS measurement noise");

      ImGui::Text("Rx : %.4f", lkf_rx);

      ImGui::Text("Ry : %.4f", lkf_ry);
    }

    // --------------------------------------------------------
    // EKF parameters
    // --------------------------------------------------------

    if (simulation_mode == kf::SimulationMode::EKF) {
      ImGui::Text("Extended Kalman Filter");

      ImGui::Spacing();

      ImGui::Text("Process noise");

      ImGui::Text("Qx   : %.5f", ekf_qx);

      ImGui::Text("Qy   : %.5f", ekf_qy);

      ImGui::Text("Qv   : %.5f", ekf_qv);

      ImGui::Text("Qpsi : %.5f", ekf_qpsi);

      ImGui::Spacing();
      ImGui::Separator();
      ImGui::Spacing();

      ImGui::Text("Multi-rate Sensor Fusion");

      const kf::Scalar prediction_rate_hz = 1.0 / simulation_dt_callback();

      ImGui::Text("Prediction : %.1f Hz", prediction_rate_hz);

      ImGui::Text("GPS        : %.1f Hz", ekf_gps_rate_hz);

      ImGui::Text("LiDAR      : %.1f Hz", ekf_lidar_rate_hz);

      ImGui::Spacing();

      ImGui::Text("LiDAR noise");

      ImGui::Text("Range std   : %.3f m", lidar_range_std);

      ImGui::Text("Bearing std : %.4f rad", lidar_bearing_std);

      ImGui::Spacing();

      ImGui::Text("Landmark");

      ImGui::Text("X : %.2f m", landmark_x);

      ImGui::Text("Y : %.2f m", landmark_y);
    }

    // ========================================================
    // CONTROLS
    // ========================================================

    ImGui::Spacing();

    ImGui::Separator();

    ImGui::Spacing();

    if (ImGui::Button("Start")) {
      if (simulation_active) {
        simulation_running = true;

        last_step_time = glfwGetTime();
      }
    }

    ImGui::SameLine();

    if (ImGui::Button("Pause")) {
      simulation_running = false;
    }

    ImGui::SameLine();

    if (ImGui::Button("Reset")) {
      simulation_running = false;

      history.clear();

      reset_callback();

      simulation_active = true;

      last_step_time = glfwGetTime();

      // Reset means start again from t = 0.
      simulation_running = true;
    }

    ImGui::Spacing();

    if (!simulation_active) {
      ImGui::Text("Status: COMPLETE");
    } else {
      ImGui::Text("Status: %s", simulation_running ? "RUNNING" : "PAUSED");
    }

    // ========================================================
    // SIMULATION INFORMATION
    // ========================================================

    ImGui::Spacing();

    ImGui::Separator();

    ImGui::Spacing();

    ImGui::Text("Simulation samples: %zu", history.size());

    if (!history.empty()) {
      ImGui::Text("Simulation time: %.2f s", history.back().time);
    }

    ImGui::EndChild();

    // ========================================================
    // BOTTOM LEFT
    // POSITION ERROR
    // ========================================================

    ImGui::BeginChild("PositionErrorPanel", ImVec2(left_width, bottom_height),
                      true);

    ImGui::Text("Position Error vs Time");

    ImGui::Separator();

    const ImVec2 error_plot_size = ImGui::GetContentRegionAvail();

    if (error_plot_size.x > 10.0f && error_plot_size.y > 10.0f) {
      if (ImPlot::BeginPlot("##PositionError", error_plot_size)) {
        ImPlot::SetupAxis(ImAxis_X1, "Time [s]", ImPlotAxisFlags_AutoFit);

        ImPlot::SetupAxis(ImAxis_Y1, "Position error [m]",
                          ImPlotAxisFlags_AutoFit);

        if (!time_data.empty()) {
          ImPlot::PlotLine("ex", time_data.data(), error_x.data(),
                           static_cast<int>(time_data.size()));

          ImPlot::PlotLine("ey", time_data.data(), error_y.data(),
                           static_cast<int>(time_data.size()));
        }

        ImPlot::EndPlot();
      }
    }

    ImGui::EndChild();

    // ========================================================
    // BOTTOM RIGHT
    // FILTER DIAGNOSTICS
    // ========================================================

    ImGui::SameLine();

    ImGui::BeginChild("DiagnosticsPanel", ImVec2(right_width, bottom_height),
                      true);

    ImGui::Text("Filter Diagnostics");

    ImGui::Separator();

    if (!history.empty()) {
      const auto &latest = history.back();

      const double trace_p = latest.estimated_covariance.trace();

      // ====================================================
      // SENSOR ACTIVITY
      // ====================================================

      if (simulation_mode == kf::SimulationMode::EKF) {
        ImGui::Text("Sensors");

        ImGui::Text("GPS   : %s", latest.gps_enabled ? "ENABLED" : "DISABLED");

        ImGui::Text("LiDAR : %s",
                    latest.lidar_enabled ? "ENABLED" : "DISABLED");

        ImGui::Spacing();

        ImGui::Text("Updates this prediction step");

        ImGui::Text("GPS   : %s", latest.gps_updated ? "UPDATE" : "--");

        ImGui::Text("LiDAR : %s", latest.lidar_updated ? "UPDATE" : "--");

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
      }

      // ====================================================
      // INNOVATION
      // ====================================================

      if (simulation_mode == kf::SimulationMode::LKF) {
        // ------------------------------------------------
        // LKF innovation
        // ------------------------------------------------

        ImGui::Text("Innovation");

        if (latest.innovation.size() >= 2) {
          ImGui::Text("X : %.4f m", latest.innovation(0));

          ImGui::Text("Y : %.4f m", latest.innovation(1));
        }
      } else if (simulation_mode == kf::SimulationMode::EKF) {
        // ------------------------------------------------
        // GPS innovation
        // ------------------------------------------------

        if (latest.gps_updated && latest.gps_innovation.size() >= 2) {
          ImGui::Text("GPS Innovation");

          ImGui::Text("X : %.4f m", latest.gps_innovation(0));

          ImGui::Text("Y : %.4f m", latest.gps_innovation(1));

          ImGui::Spacing();
        }

        // ------------------------------------------------
        // LiDAR innovation
        // ------------------------------------------------

        if (latest.lidar_updated && latest.lidar_innovation.size() >= 2) {
          ImGui::Text("LiDAR Innovation");

          ImGui::Text("Range   : %.4f m", latest.lidar_innovation(0));

          ImGui::Text("Bearing : %.5f rad", latest.lidar_innovation(1));

          ImGui::Spacing();
        }
      }
      // ====================================================
      // KALMAN GAIN
      // ====================================================

      ImGui::Spacing();

      ImGui::Separator();

      ImGui::Spacing();

      ImGui::Text("Kalman Gain K");

      if (latest.kalman_gain.rows() == 4 && latest.kalman_gain.cols() == 2) {
        for (int row = 0; row < 4; ++row) {
          ImGui::Text("[ %8.4f  %8.4f ]", latest.kalman_gain(row, 0),
                      latest.kalman_gain(row, 1));
        }
      }

      // ====================================================
      // TRACE P
      // ====================================================

      ImGui::Spacing();

      ImGui::Separator();

      ImGui::Spacing();

      ImGui::Text("Trace(P) : %.6f", trace_p);

      // ====================================================
      // CURRENT STATE
      // ====================================================

      ImGui::Spacing();

      ImGui::Separator();

      ImGui::Spacing();

      ImGui::Text("Current State");

      // ----------------------------------------------------
      // LKF state:
      // [px, py, vx, vy]
      // ----------------------------------------------------

      if (simulation_mode == kf::SimulationMode::LKF) {
        ImGui::Text("px : %.3f m", latest.estimated_state(0));

        ImGui::Text("py : %.3f m", latest.estimated_state(1));

        ImGui::Text("vx : %.3f m/s", latest.estimated_state(2));

        ImGui::Text("vy : %.3f m/s", latest.estimated_state(3));
      }

      // ----------------------------------------------------
      // EKF state:
      // [X, Y, v, psi]
      // ----------------------------------------------------

      if (simulation_mode == kf::SimulationMode::EKF) {
        ImGui::Text("X   : %.3f m", latest.estimated_state(0));

        ImGui::Text("Y   : %.3f m", latest.estimated_state(1));

        ImGui::Text("v   : %.3f m/s", latest.estimated_state(2));

        ImGui::Text("psi : %.5f rad", latest.estimated_state(3));
      }

      // ====================================================
      // RMSE
      // ====================================================

      ImGui::Spacing();

      ImGui::Separator();

      ImGui::Spacing();

      ImGui::Text("Position RMSE");

      ImGui::Text("X : %.4f m", rmse_x);

      ImGui::Text("Y : %.4f m", rmse_y);

      ImGui::Text("2D: %.4f m", rmse_position);
    } else {
      ImGui::Text("Waiting for simulation data...");
    }

    ImGui::EndChild();

    // ========================================================
    // END MAIN WINDOW
    // ========================================================

    ImGui::End();

    // ========================================================
    // FINISH IMGUI FRAME
    // ========================================================

    ImGui::Render();

    // ========================================================
    // OPENGL RENDERING
    // ========================================================

    int display_width = 0;

    int display_height = 0;

    glfwGetFramebufferSize(window_, &display_width, &display_height);

    glViewport(0, 0, display_width, display_height);

    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

    glClear(GL_COLOR_BUFFER_BIT);

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window_);
  }
}