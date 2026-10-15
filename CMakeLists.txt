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

add_executable(
    kalman_demo
    app/main.cpp
    src/models/ConstantVelocityModel.cpp
    src/filters/LinearKalmanFilter.cpp
    src/models/GPSMeasurementModel.cpp
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
)