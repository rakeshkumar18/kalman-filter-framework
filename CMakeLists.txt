cmake_minimum_required(VERSION 3.16)

project(
    KalmanFilterFramework
    VERSION 0.1
    LANGUAGES CXX
)

add_executable(
    kalman_demo
    app/main.cpp
)

target_compile_features(
    kalman_demo
    PRIVATE
    cxx_std_17
)