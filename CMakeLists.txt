cmake_minimum_required(VERSION 3.16)

project(
    KalmanFilterFramework
    VERSION 0.1
    LANGUAGES CXX
)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(
    Eigen3 3.4
    REQUIRED
    NO_MODULE
)


find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

# ---------------------------------------------------------
# Dear ImGui
# ---------------------------------------------------------

add_library(
    imgui
    STATIC

    external/imgui/imgui.cpp
    external/imgui/imgui_draw.cpp
    external/imgui/imgui_tables.cpp
    external/imgui/imgui_widgets.cpp

    external/imgui/backends/imgui_impl_glfw.cpp
    external/imgui/backends/imgui_impl_opengl3.cpp
)

target_include_directories(
    imgui
    PUBLIC

    ${PROJECT_SOURCE_DIR}/external/imgui
    ${PROJECT_SOURCE_DIR}/external/imgui/backends
)

target_link_libraries(
    imgui
    PUBLIC

    glfw
    OpenGL::GL
)

# ---------------------------------------------------------
# ImPlot
# ---------------------------------------------------------

add_library(
    implot
    STATIC

    external/implot/implot.cpp
    external/implot/implot_items.cpp
)

target_include_directories(
    implot
    PUBLIC

    ${PROJECT_SOURCE_DIR}/external/implot
)

target_link_libraries(
    implot
    PUBLIC

    imgui
)


add_executable(
    kalman_demo
    app/main.cpp
    src/models/ConstantVelocityModel.cpp
    src/filters/LinearKalmanFilter.cpp
    src/models/GPSMeasurementModel.cpp
    src/simulation/VehicleSimulator.cpp
    src/sensors/GPSSensor.cpp
    src/visualization/RealTimeViewer.cpp
)


target_include_directories(
    kalman_demo
    PRIVATE
    ${PROJECT_SOURCE_DIR}/include()
)
target_compile_features(
    kalman_demo
    PRIVATE
    cxx_std_17
)

target_link_libraries(
    kalman_demo
    PRIVATE
    Eigen3::Eigen
    implot
)

if(APPLE)
    target_compile_definitions(
        imgui
        PUBLIC
        GL_SILENCE_DEPRECATION
    )
endif()