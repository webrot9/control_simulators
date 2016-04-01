#include "cartpole.h"
#include <iostream>

// Cartpole state is: position, velocity, angle, angular velocity;
// for the angle, 0 is straigth up, PI is straigth down
ConsistentVector Cartpole::dynamics(double time,
				    const ConsistentVector& state,
				    const ConsistentVector &control) const {
  checkStateSize(state);
  checkControlSize(control);
  ConsistentVector velocities(state.size());

  double sint = std::sin(state[2]);
  double cost = std::cos(state[2]);

  double g = 9.81;
  double M = param_[CART_MASS];
  double m = param_[POLE_MASS];
  double l = param_[POLE_LENGTH];
  double f = param_[CART_POLE_FRICTION];

  double div = M + (1 - cost*cost)*m;

  velocities[0] = state[1];
  velocities[1] = (-m*g*sint*cost + m*l*state[3]*state[3]*sint + f*m*state[3]*cost + control[0])/div;
  velocities[2] = state[3];
  velocities[3] = ((M + m)*(g*sint - f*state[3]) - (l*m*state[3]*state[3]*sint + control[0])*cost)/(l*div);

  return velocities;
}


