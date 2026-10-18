#include "kf/visualization/RealTimeViewer.hpp"

#include <GLFW/glfw3.h>
#include <cmath>
#include <Eigen/Eigenvalues>

#include "imgui.h"
#include "implot.h"

#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

namespace
{

void buildCovarianceEllipse
(
    const kf::StateVector& center,
    const kf::StateMatrix& cov,
    std::vector<double>& ellipse_x,
    std::vector<double>& ellipse_y,
    double sigma_scale = 2.0
)
{
    Eigen::Matrix2d position_cov = cov.block<2, 2>(0, 0);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(position_cov);

    if(solver.info() != Eigen::Success)
    {
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

    for (int i = 0; i <= number_of_points; ++i)
    {
        double theta =
            2.0 * M_PI *
            static_cast<double>(i) /
            static_cast<double>(number_of_points);


        Eigen::Vector2d local_point;

        local_point(0) =
            sigma_scale *
            std::sqrt(eigenvalues(0)) *
            std::cos(theta);

        local_point(1) =
            sigma_scale *
            std::sqrt(eigenvalues(1)) *
            std::sin(theta);


        Eigen::Vector2d rotated_point =
            eigenvectors * local_point;


        ellipse_x.push_back(
            center(0) + rotated_point(0)
        );

        ellipse_y.push_back(
            center(1) + rotated_point(1)
        );
    }


}

}


kf::RealTimeViewer::RealTimeViewer()
    : window_(nullptr),
      initialized_(false)
{
}


kf::RealTimeViewer::~RealTimeViewer()
{
    shutdown();
}


bool
kf::RealTimeViewer::initialize()
{
    // --------------------------------------------
    // 1. Initialize GLFW
    // --------------------------------------------
    if (!glfwInit())
    {
        return false;
    }


    #ifdef __APPLE__

        glfwWindowHint(
            GLFW_CONTEXT_VERSION_MAJOR,
            3
        );

        glfwWindowHint(
            GLFW_CONTEXT_VERSION_MINOR,
            2
        );

        glfwWindowHint(
            GLFW_OPENGL_PROFILE,
            GLFW_OPENGL_CORE_PROFILE
        );

        glfwWindowHint(
            GLFW_OPENGL_FORWARD_COMPAT,
            GL_TRUE
        );

    #endif


    // --------------------------------------------
    // 2. Create window
    // --------------------------------------------
    window_ = glfwCreateWindow(
        1280,
        720,
        "Kalman Filter Simulation",
        nullptr,
        nullptr
    );

    if (window_ == nullptr)
    {
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
    if (!ImGui_ImplGlfw_InitForOpenGL(
            window_,
            true
        ))
    {
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
    const char* glsl_version =
        "#version 150";

    if (!ImGui_ImplOpenGL3_Init(
            glsl_version
        ))
    {
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


void
kf::RealTimeViewer::shutdown()
{
    if (!initialized_)
    {
        return;
    }


    // Reverse order of initialization
    ImGui_ImplOpenGL3_Shutdown();

    ImGui_ImplGlfw_Shutdown();

    ImPlot::DestroyContext();

    ImGui::DestroyContext();


    if (window_ != nullptr)
    {
        glfwDestroyWindow(window_);

        window_ = nullptr;
    }


    glfwTerminate();

    initialized_ = false;
}

void
kf::RealTimeViewer::run(
    const std::vector<kf::EstimatorSnapshot>& history
)
{
    while (!glfwWindowShouldClose(window_))
    {
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

        ImGui::SetNextWindowPos(
            ImVec2(0.0f, 0.0f),
            ImGuiCond_Always
        );

        ImGui::SetNextWindowSize(
            ImGui::GetIO().DisplaySize,
            ImGuiCond_Always
        );

        ImGui::Begin(
            "Kalman Filter Simulation",
            nullptr,
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoCollapse
        );


        // --------------------------------------------------
        // Available space inside main window
        // --------------------------------------------------

        ImVec2 available =
            ImGui::GetContentRegionAvail();

        float left_width =
            available.x * 0.68f;

        float right_width =
            available.x - left_width;

        float top_height =
            available.y * 0.62f;

        float bottom_height =
            available.y - top_height;


        // --------------------------------------------------
        // Top-left: XY trajectory
        // --------------------------------------------------

        ImGui::BeginChild(
            "XYTrajectoryPanel",
            ImVec2(
                left_width,
                top_height
            ),
            true
        );

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
                
        for (const auto& snapshot : history)
        {
            true_x.push_back(
                snapshot.true_state(0)
            );

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

        if (ImPlot::BeginPlot(
                "##XYTrajectory",
                ImVec2(-1, -1)
            ))
        {
            ImPlot::SetupAxes(
                "X position [m]",
                "Y position [m]"
            );

            ImPlot::PlotLine(
                "Truth",
                true_x.data(),
                true_y.data(),
                static_cast<int>(true_x.size())
            );

            ImPlot::PlotScatter(
                "GPS",
                gps_x.data(),
                gps_y.data(),
                static_cast<int>(gps_x.size())
            );
            ImPlot::PlotLine(
                "KF Prediction",
                predicted_x.data(),
                predicted_y.data(),
                static_cast<int>(predicted_x.size())
            );

            ImPlot::PlotLine(
                "KF Estimate",
                estimate_x.data(),
                estimate_y.data(),
                static_cast<int>(estimate_x.size())
            );

            

            ImPlot::EndPlot();
        }


        ImGui::EndChild();


        // Same row
        ImGui::SameLine();


        // --------------------------------------------------
        // Top-right: Filter parameters
        // --------------------------------------------------

        ImGui::BeginChild(
            "FilterParametersPanel",
            ImVec2(
                right_width,
                top_height
            ),
            true
        );

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

        ImGui::BeginChild(
            "PositionErrorPanel",
            ImVec2(
                left_width,
                bottom_height
            ),
            true
        );

        ImGui::Text("Position Error vs Time");
        ImGui::Separator();

        ImVec2 error_plot_size =
            ImGui::GetContentRegionAvail();

        if (
            error_plot_size.x > 10.0f &&
            error_plot_size.y > 10.0f
        )
        {
            if (
                ImPlot::BeginPlot(
                    "##PositionError",
                    error_plot_size
                )
            )
            {
                ImPlot::SetupAxis(
                    ImAxis_X1,
                    "Time [s]",
                    ImPlotAxisFlags_AutoFit
                );

                ImPlot::SetupAxis(
                    ImAxis_Y1,
                    "Position error [m]",
                    ImPlotAxisFlags_AutoFit
                );

                if (
                    !time_data.empty() &&
                    time_data.size() == error_x.size() &&
                    time_data.size() == error_y.size()
                )
                {
                    ImPlot::PlotLine(
                        "ex",
                        time_data.data(),
                        error_x.data(),
                        static_cast<int>(time_data.size())
                    );

                    ImPlot::PlotLine(
                        "ey",
                        time_data.data(),
                        error_y.data(),
                        static_cast<int>(time_data.size())
                    );
                }

                ImPlot::EndPlot();
            }
        }

        ImGui::EndChild();

        // --------------------------------------------------
        // Bottom-right: Diagnostics
        // --------------------------------------------------

        ImGui::BeginChild(
            "DiagnosticsPanel",
            ImVec2(
                right_width,
                bottom_height
            ),
            true
        );

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

        glfwGetFramebufferSize(
            window_,
            &display_width,
            &display_height
        );

        glViewport(
            0,
            0,
            display_width,
            display_height
        );


        // --------------------------------------------
        // 6. Clear background
        // --------------------------------------------
        glClearColor(
            0.1f,
            0.1f,
            0.1f,
            1.0f
        );

        glClear(
            GL_COLOR_BUFFER_BIT
        );


        // --------------------------------------------
        // 7. Render ImGui
        // --------------------------------------------
        ImGui_ImplOpenGL3_RenderDrawData(
            ImGui::GetDrawData()
        );


        // --------------------------------------------
        // 8. Display frame
        // --------------------------------------------
        glfwSwapBuffers(window_);
    }
}

void
kf::RealTimeViewer::runLive(
    std::vector<kf::EstimatorSnapshot>& history,
    const kf::RealTimeViewer::StepCallback& step_callback,
    kf::Scalar sim_dt,
    kf::Scalar qx,
    kf::Scalar qy,
    kf::Scalar rx,
    kf::Scalar ry
)
{
    // ==================================================
    // Simulation state
    // ==================================================

    double last_step_time =
        glfwGetTime();

    bool simulation_active =
        true;

    bool simulation_running =
        false;


    // ==================================================
    // MAIN APPLICATION LOOP
    // ==================================================

    while (!glfwWindowShouldClose(window_))
    {
        // --------------------------------------------------
        // 1. GLFW events
        // --------------------------------------------------

        glfwPollEvents();


        // --------------------------------------------------
        // 2. Advance simulation
        // --------------------------------------------------

        const double current_time =
            glfwGetTime();


        if (
            simulation_active &&
            simulation_running &&
            (current_time - last_step_time) >= sim_dt
        )
        {
            kf::EstimatorSnapshot snapshot;


            if (step_callback(snapshot))
            {
                history.push_back(snapshot);
            }
            else
            {
                simulation_active =
                    false;

                simulation_running =
                    false;
            }


            last_step_time =
                current_time;
        }


        // ==================================================
        // 3. START IMGUI FRAME
        // ==================================================

        ImGui_ImplOpenGL3_NewFrame();

        ImGui_ImplGlfw_NewFrame();

        ImGui::NewFrame();


        // ==================================================
        // 4. MAIN WINDOW
        // ==================================================

        ImGui::SetNextWindowPos(
            ImVec2(0.0f, 0.0f),
            ImGuiCond_Always
        );


        ImGui::SetNextWindowSize(
            ImGui::GetIO().DisplaySize,
            ImGuiCond_Always
        );


        ImGui::Begin(
            "Kalman Filter Simulation",
            nullptr,
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoCollapse
        );


        // --------------------------------------------------
        // Dashboard geometry
        // --------------------------------------------------

        const ImVec2 available =
            ImGui::GetContentRegionAvail();


        const float horizontal_spacing =
            ImGui::GetStyle().ItemSpacing.x;


        const float vertical_spacing =
            ImGui::GetStyle().ItemSpacing.y;


        const float left_width =
            available.x * 0.68f;


        const float right_width =
            available.x
            - left_width
            - horizontal_spacing;


        const float top_height =
            available.y * 0.62f;


        const float bottom_height =
            available.y
            - top_height
            - vertical_spacing;


        // ==================================================
        // PREPARE DATA
        // ==================================================

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


        const std::size_t sample_count =
            history.size();


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


        for (const auto& snapshot : history)
        {
            // Truth
            true_x.push_back(
                snapshot.true_state(0)
            );

            true_y.push_back(
                snapshot.true_state(1)
            );


            // GPS
            gps_x.push_back(
                snapshot.gps_measurement(0)
            );

            gps_y.push_back(
                snapshot.gps_measurement(1)
            );


            // Prediction x-
            predicted_x.push_back(
                snapshot.predicted_state(0)
            );

            predicted_y.push_back(
                snapshot.predicted_state(1)
            );


            // Posterior x+
            estimate_x.push_back(
                snapshot.estimated_state(0)
            );

            estimate_y.push_back(
                snapshot.estimated_state(1)
            );


            // Time
            time_data.push_back(
                snapshot.time
            );


            // Posterior position errors
            error_x.push_back(
                snapshot.estimated_state(0)
                - snapshot.true_state(0)
            );

            error_y.push_back(
                snapshot.estimated_state(1)
                - snapshot.true_state(1)
            );
        }


        // ==================================================
        // COVARIANCE ELLIPSES
        // latest sample only
        // ==================================================

        std::vector<double> prior_ellipse_x;
        std::vector<double> prior_ellipse_y;

        std::vector<double> posterior_ellipse_x;
        std::vector<double> posterior_ellipse_y;


        if (!history.empty())
        {
            const auto& latest =
                history.back();


            buildCovarianceEllipse(
                latest.predicted_state,
                latest.predicted_covariance,
                prior_ellipse_x,
                prior_ellipse_y
            );


            buildCovarianceEllipse(
                latest.estimated_state,
                latest.estimated_covariance,
                posterior_ellipse_x,
                posterior_ellipse_y
            );
        }


        // ==================================================
        // RMSE
        // ==================================================

        double rmse_x =
            0.0;

        double rmse_y =
            0.0;

        double rmse_position =
            0.0;


        if (!history.empty())
        {
            double sum_squared_error_x =
                0.0;

            double sum_squared_error_y =
                0.0;


            for (
                std::size_t i = 0;
                i < error_x.size();
                ++i
            )
            {
                sum_squared_error_x +=
                    error_x[i] * error_x[i];


                sum_squared_error_y +=
                    error_y[i] * error_y[i];
            }


            const double n =
                static_cast<double>(
                    error_x.size()
                );


            if (n > 0.0)
            {
                rmse_x =
                    std::sqrt(
                        sum_squared_error_x / n
                    );


                rmse_y =
                    std::sqrt(
                        sum_squared_error_y / n
                    );


                rmse_position =
                    std::sqrt(
                        (
                            sum_squared_error_x
                            +
                            sum_squared_error_y
                        )
                        /
                        n
                    );
            }
        }


        // ==================================================
        // TOP-LEFT
        // XY TRAJECTORY
        // ==================================================

        ImGui::BeginChild(
            "XYTrajectoryPanel",
            ImVec2(
                left_width,
                top_height
            ),
            true
        );


        ImGui::Text(
            "XY Trajectory"
        );


        ImGui::Separator();


        if (
            ImPlot::BeginPlot(
                "##XYTrajectory",
                ImVec2(-1, -1)
            )
        )
        {
            ImPlot::SetupAxis(
                ImAxis_X1,
                "X position [m]",
                ImPlotAxisFlags_AutoFit
            );


            ImPlot::SetupAxis(
                ImAxis_Y1,
                "Y position [m]",
                ImPlotAxisFlags_AutoFit
            );


            if (!history.empty())
            {
                // Truth
                ImPlot::PlotLine(
                    "Truth",
                    true_x.data(),
                    true_y.data(),
                    static_cast<int>(
                        true_x.size()
                    )
                );


                // GPS
                ImPlot::PlotScatter(
                    "GPS",
                    gps_x.data(),
                    gps_y.data(),
                    static_cast<int>(
                        gps_x.size()
                    )
                );


                // Prediction
                ImPlot::PlotLine(
                    "KF Prediction",
                    predicted_x.data(),
                    predicted_y.data(),
                    static_cast<int>(
                        predicted_x.size()
                    )
                );


                // Posterior estimate
                ImPlot::PlotLine(
                    "KF Estimate",
                    estimate_x.data(),
                    estimate_y.data(),
                    static_cast<int>(
                        estimate_x.size()
                    )
                );


                // Prior ellipse
                if (!prior_ellipse_x.empty())
                {
                    ImPlot::PlotLine(
                        "Prior 2-sigma",
                        prior_ellipse_x.data(),
                        prior_ellipse_y.data(),
                        static_cast<int>(
                            prior_ellipse_x.size()
                        )
                    );
                }


                // Posterior ellipse
                if (!posterior_ellipse_x.empty())
                {
                    ImPlot::PlotLine(
                        "Posterior 2-sigma",
                        posterior_ellipse_x.data(),
                        posterior_ellipse_y.data(),
                        static_cast<int>(
                            posterior_ellipse_x.size()
                        )
                    );
                }
            }


            ImPlot::EndPlot();
        }


        ImGui::EndChild();


        // ==================================================
        // TOP-RIGHT
        // FILTER PARAMETERS
        // ==================================================

        ImGui::SameLine();


        ImGui::BeginChild(
            "FilterParametersPanel",
            ImVec2(
                right_width,
                top_height
            ),
            true
        );


        ImGui::Text(
            "Filter Parameters"
        );


        ImGui::Separator();


        // --------------------------------------------------
        // Q / R
        // --------------------------------------------------

        ImGui::Text(
            "Qx : %.4f",
            qx
        );


        ImGui::Text(
            "Qy : %.4f",
            qy
        );


        ImGui::Spacing();


        ImGui::Text(
            "Rx : %.4f",
            rx
        );


        ImGui::Text(
            "Ry : %.4f",
            ry
        );


        // --------------------------------------------------
        // Simulation controls
        // --------------------------------------------------

        ImGui::Spacing();

        ImGui::Separator();

        ImGui::Spacing();


        if (ImGui::Button("Start"))
        {
            if (simulation_active)
            {
                simulation_running =
                    true;


                last_step_time =
                    glfwGetTime();
            }
        }


        ImGui::SameLine();


        if (ImGui::Button("Pause"))
        {
            simulation_running =
                false;
        }


        ImGui::Spacing();


        if (!simulation_active)
        {
            ImGui::Text(
                "Status: COMPLETE"
            );
        }
        else
        {
            ImGui::Text(
                "Status: %s",
                simulation_running
                    ? "RUNNING"
                    : "PAUSED"
            );
        }


        // --------------------------------------------------
        // Simulation information
        // --------------------------------------------------

        ImGui::Spacing();

        ImGui::Separator();

        ImGui::Spacing();


        ImGui::Text(
            "Simulation samples: %zu",
            history.size()
        );


        if (!history.empty())
        {
            ImGui::Text(
                "Simulation time: %.2f s",
                history.back().time
            );
        }


        ImGui::EndChild();


        // ==================================================
        // BOTTOM-LEFT
        // POSITION ERROR
        // ==================================================

        ImGui::BeginChild(
            "PositionErrorPanel",
            ImVec2(
                left_width,
                bottom_height
            ),
            true
        );


        ImGui::Text(
            "Position Error vs Time"
        );


        ImGui::Separator();


        const ImVec2 error_plot_size =
            ImGui::GetContentRegionAvail();


        if (
            error_plot_size.x > 10.0f &&
            error_plot_size.y > 10.0f
        )
        {
            if (
                ImPlot::BeginPlot(
                    "##PositionError",
                    error_plot_size
                )
            )
            {
                ImPlot::SetupAxis(
                    ImAxis_X1,
                    "Time [s]",
                    ImPlotAxisFlags_AutoFit
                );


                ImPlot::SetupAxis(
                    ImAxis_Y1,
                    "Position error [m]",
                    ImPlotAxisFlags_AutoFit
                );


                if (!time_data.empty())
                {
                    ImPlot::PlotLine(
                        "ex",
                        time_data.data(),
                        error_x.data(),
                        static_cast<int>(
                            time_data.size()
                        )
                    );


                    ImPlot::PlotLine(
                        "ey",
                        time_data.data(),
                        error_y.data(),
                        static_cast<int>(
                            time_data.size()
                        )
                    );
                }


                ImPlot::EndPlot();
            }
        }


        ImGui::EndChild();


        // ==================================================
        // BOTTOM-RIGHT
        // FILTER DIAGNOSTICS
        // ==================================================

        ImGui::SameLine();


        ImGui::BeginChild(
            "DiagnosticsPanel",
            ImVec2(
                right_width,
                bottom_height
            ),
            true
        );


        ImGui::Text(
            "Filter Diagnostics"
        );


        ImGui::Separator();


        if (!history.empty())
        {
            const auto& latest =
                history.back();


            const double trace_p =
                latest.estimated_covariance.trace();


            // --------------------------------------------------
            // Innovation
            // --------------------------------------------------

            ImGui::Text(
                "Innovation"
            );


            if (
                latest.innovation.size()
                >= 2
            )
            {
                ImGui::Text(
                    "yx : %.4f m",
                    latest.innovation(0)
                );


                ImGui::Text(
                    "yy : %.4f m",
                    latest.innovation(1)
                );
            }


            // --------------------------------------------------
            // Kalman Gain
            // --------------------------------------------------

            ImGui::Spacing();

            ImGui::Separator();

            ImGui::Spacing();


            ImGui::Text(
                "Kalman Gain K"
            );


            if (
                latest.kalman_gain.rows() == 4 &&
                latest.kalman_gain.cols() == 2
            )
            {
                for (
                    int row = 0;
                    row < 4;
                    ++row
                )
                {
                    ImGui::Text(
                        "[ %7.4f  %7.4f ]",
                        latest.kalman_gain(
                            row,
                            0
                        ),
                        latest.kalman_gain(
                            row,
                            1
                        )
                    );
                }
            }


            // --------------------------------------------------
            // Trace P
            // --------------------------------------------------

            ImGui::Spacing();

            ImGui::Separator();

            ImGui::Spacing();


            ImGui::Text(
                "Trace(P) : %.5f",
                trace_p
            );


            // --------------------------------------------------
            // Current state
            // --------------------------------------------------

            ImGui::Spacing();

            ImGui::Separator();

            ImGui::Spacing();


            ImGui::Text(
                "Current State"
            );


            ImGui::Text(
                "px : %.3f m",
                latest.estimated_state(0)
            );


            ImGui::Text(
                "py : %.3f m",
                latest.estimated_state(1)
            );


            ImGui::Text(
                "vx : %.3f m/s",
                latest.estimated_state(2)
            );


            ImGui::Text(
                "vy : %.3f m/s",
                latest.estimated_state(3)
            );


            // --------------------------------------------------
            // RMSE
            // --------------------------------------------------

            ImGui::Spacing();

            ImGui::Separator();

            ImGui::Spacing();


            ImGui::Text(
                "RMSE"
            );


            ImGui::Text(
                "X position  : %.4f m",
                rmse_x
            );


            ImGui::Text(
                "Y position  : %.4f m",
                rmse_y
            );


            ImGui::Text(
                "2D position : %.4f m",
                rmse_position
            );
        }
        else
        {
            ImGui::Text(
                "Waiting for simulation data..."
            );
        }


        ImGui::EndChild();


        // ==================================================
        // END MAIN WINDOW
        // ==================================================

        ImGui::End();


        // ==================================================
        // 5. FINISH IMGUI FRAME
        // ==================================================

        ImGui::Render();


        // ==================================================
        // 6. OPENGL RENDERING
        // ==================================================

        int display_width =
            0;

        int display_height =
            0;


        glfwGetFramebufferSize(
            window_,
            &display_width,
            &display_height
        );


        glViewport(
            0,
            0,
            display_width,
            display_height
        );


        glClearColor(
            0.1f,
            0.1f,
            0.1f,
            1.0f
        );


        glClear(
            GL_COLOR_BUFFER_BIT
        );


        ImGui_ImplOpenGL3_RenderDrawData(
            ImGui::GetDrawData()
        );


        glfwSwapBuffers(
            window_
        );
    }
}