#pragma once

#include <random>

#include "kf/core/Types.hpp"

namespace kf
{

class GPSSensor
{
private:
    std::mt19937 generator_;
    std::normal_distribution<Scalar> noise_x_;
    std::normal_distribution<Scalar> noise_y_;

public:
    GPSSensor(Scalar std_x, Scalar std_y);
    MeasurementVector measure(const StateVector& true_state);
    
};

}