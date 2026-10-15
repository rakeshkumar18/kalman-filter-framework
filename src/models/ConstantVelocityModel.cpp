#include "kf/models/ConstantVelocityModel.hpp"



kf::StateMatrix kf::ConstantVelocityModel::stateTransition(kf::Scalar dt) const
{
    kf::StateMatrix F = kf::StateMatrix::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;

    return F;
}

kf::StateMatrix kf::ConstantVelocityModel::processNoise(kf::Scalar dt, const kf::AccelerationCovariance &acc_cov) const
{
    kf::ProcessNoiseMapping G = kf::ProcessNoiseMapping::Zero();

    G(0,0) = 0.5 * dt * dt;
    G(1, 1) = 0.5 * dt * dt;
    G(2, 0) = dt;
    G(3, 1) = dt;

    kf::StateMatrix Q =  G * acc_cov * G.transpose();

    return Q;

}


