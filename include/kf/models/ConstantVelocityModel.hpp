# pragma once

#include "kf/core/Types.hpp"

namespace kf
{

class ConstantVelocityModel
{
public:
    StateMatrix stateTransition(Scalar dt) const;

    StateMatrix processNoise(Scalar dt, const AccelerationCovariance &acc_cov) const;
};

}
