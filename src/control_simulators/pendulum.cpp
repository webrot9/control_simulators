#include "pendulum.h"

ConsistentVector Pendulum::dynamics(double time,
				    const ConsistentVector& state,
				    const ConsistentVector &control) const {
  checkStateSize(state);
  checkControlSize(control);
  ConsistentVector velocities(state.size());

  velocities[0] = state[1];
  velocities[1] = -9.81*std::sin(state[0])/param_[LENGTH] - param_[DAMPING_COEFF]*state[1] + control[0];

  return velocities;
}


