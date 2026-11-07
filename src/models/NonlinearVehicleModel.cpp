#include "kf/models/NonlinearVehicleModel.hpp"

#include <cmath>

kf::NonlinearVehicleModel::NonlinearVehicleModel(
    const kf::VehicleParameters &parameters)
    : parameters_(parameters) {}

// ============================================================
// CG sideslip angle
//
// beta = atan(
//     lr / (lf + lr) * tan(delta)
// )
// ============================================================

kf::Scalar
kf::NonlinearVehicleModel::sideslipAngle(kf::Scalar steering_angle) const {
  const kf::Scalar wheelbase = parameters_.lf + parameters_.lr;

  const kf::Scalar beta =
      std::atan((parameters_.lr / wheelbase) * std::tan(steering_angle));

  return beta;
}

// ============================================================
// Nonlinear state propagation
//
// state = [X, Y, v, psi]
//
// input = [a, delta]
// ============================================================

kf::StateVector
kf::NonlinearVehicleModel::propagate(const kf::StateVector &state,
                                     const kf::VehicleControlInput &input,
                                     kf::Scalar dt) const {
  //====================================================
  // Current State
  //====================================================

  const kf::Scalar X = state(0);
  const kf::Scalar Y = state(1);
  const kf::Scalar v = state(2);
  const kf::Scalar psi = state(3);

  //====================================================
  // Input
  //====================================================

  const kf::Scalar acceleration = input.acceleration;
  const kf::Scalar steering_angle = input.steering_angle;

  // --------------------------------------------------------
  // Vehicle sideslip
  // --------------------------------------------------------

  const kf::Scalar beta = sideslipAngle(steering_angle);

  // --------------------------------------------------------
  // Next state
  // --------------------------------------------------------

  kf::StateVector next_state;

  next_state(0) = X + v * std::cos(psi + beta) * dt;
  next_state(1) = Y + v * std::sin(psi + beta) * dt;
  next_state(2) = v + acceleration * dt;
  next_state(3) = psi + (v / parameters_.lr) * std::sin(beta) * dt;

  return next_state;
}

// ============================================================
// Analytical state Jacobian
//
// F = df/dx
// ============================================================

kf::StateMatrix
kf::NonlinearVehicleModel::jacobian(const kf::StateVector &state,
                                    const kf::VehicleControlInput &input,
                                    kf::Scalar dt) {
  const kf::Scalar v = state(2);
  const kf::Scalar psi = state(3);
  const kf::Scalar beta = sideslipAngle(input.steering_angle);

  kf::StateMatrix F = kf::StateMatrix::Identity();

  // --------------------------------------------------------
  // dX_next / dv
  // --------------------------------------------------------

  F(0, 2) = std::cos(psi + beta) * dt;
  // --------------------------------------------------------
  // dX_next / dpsi
  // --------------------------------------------------------

  F(0, 3) = -v * std::sin(psi + beta) * dt;
  // --------------------------------------------------------
  // dY_next / dv
  // --------------------------------------------------------

  F(1, 2) = std::sin(psi + beta) * dt;
  // --------------------------------------------------------
  // dY_next / dpsi
  // --------------------------------------------------------

  F(1, 3) = v * std::cos(psi + beta) * dt;

  // --------------------------------------------------------
  // dpsi_next / dv
  // --------------------------------------------------------

  F(3, 2) = (std::sin(beta) / parameters_.lr) * dt;

  return F;
}