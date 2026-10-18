#include "kf/simulation/VehicleSimulator.hpp"

kf::VehicleSimulator::VehicleSimulator(const kf::StateVector &initial_state)
    : true_state_(initial_state)
    {

    }

void kf::VehicleSimulator::step(kf::Scalar dt, kf::Scalar ax, kf::Scalar ay)
    {
        true_state_(0) = true_state_(0) + true_state_(2) * dt
        + 0.5 * ax * dt * dt;

        true_state_(1) = true_state_(1) + true_state_(3) * dt
            + 0.5 * ay * dt * dt;

        true_state_(2) = true_state_(2) + ax * dt;

        true_state_(3) = true_state_(3) + ay * dt;

    }

const kf::StateVector &kf::VehicleSimulator::state()  const
{
    return true_state_;
}