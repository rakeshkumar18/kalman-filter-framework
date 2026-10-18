#pragma once

#include "kf/core/Types.hpp"

namespace kf
{

class VehicleSimulator
{
private:
    StateVector true_state_;

public:
    VehicleSimulator(const StateVector &intial_state);

    void step(Scalar dt, Scalar ax, Scalar ay);

    const StateVector &state() const;
};

}