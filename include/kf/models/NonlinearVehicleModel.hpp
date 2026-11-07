#pragma once
#include "kf/core/Types.hpp"

namespace kf {

//=================================================
// Control input
//
// u = [a, delta]
//=================================================

struct VehicleControlInput {
  Scalar acceleration;
  Scalar steering_angle;
};

struct VehicleParameters {
  Scalar lf; // CG -> front axle [m]
  Scalar lr; // CG -> rear axle  [m]
};

// ============================================================
// Nonlinear kinematic bicycle model
//
// State convention:
//
// x(0) = X      global X position [m]
// x(1) = Y      global Y position [m]
// x(2) = v      vehicle speed [m/s]
// x(3) = psi    heading angle [rad]
// ============================================================

class NonlinearVehicleModel {

public:
  explicit NonlinearVehicleModel(const VehicleParameters &parameters);

  StateVector propagate(const StateVector &state,
                        const VehicleControlInput &input, Scalar dt) const;

  StateMatrix jacobian(const StateVector &state,
                       const VehicleControlInput &input, Scalar dt);

private:
  // --------------------------------------------------------
  // CG sideslip angle
  //
  // beta = atan(
  //     lr / (lf + lr) * tan(delta)
  // )
  // --------------------------------------------------------

  Scalar sideslipAngle(Scalar steering_angle) const;

  VehicleParameters parameters_;
};

} // namespace kf