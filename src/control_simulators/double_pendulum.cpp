#include "double_pendulum.h"

#include <cmath>

namespace {
    constexpr double g = 9.81;
}

// State contains the following:
//  0,theta1
//  1,theta1dot
//  2,theta2
//  3,theta2dot
// Dynamics equations from: 
//   http://www.mathworks.com/matlabcentral/fileexchange/27212-animated-double-pendulum/content/double_pendulum_ODE.m
ConsistentVector DoublePendulum::dynamics(double time,
				    const ConsistentVector& state,
				    const ConsistentVector &control) const {
  checkStateSize(state);
  checkControlSize(control);

  const double m1 = param_[M1]; const double m2 = param_[M2];
  const double len1 = param_[L1]; const double len2 = param_[L2];

  const double t1 = state(0);
  const double t1dot = state(1);
  const double t2 = state(2);
  const double t2dot = state(3);

  const double t1ddot_damp = param_[D1]*t1dot;
  const double t2ddot_damp = param_[D2]*t2dot;

  const double t1_u = control[0];

  const double t1ddot = -(
          (g*(2.0*m1+m2)*sin(t1)+m2*(g*sin(t1-2*t2)+2.0*(len2*t2dot*t2dot 
               + len1*t1dot*t1dot*cos(t1-t2))*sin(t1-t2)))
          /(2.0*len1*(m1+m2-m2*cos(t1-t2)*cos(t1-t2)))
          ) - t1ddot_damp + t1_u;
  const double t2ddot = (((m1+m2)*(len1*t1dot*t1dot + g*cos(t1))+len2*m2*t2dot*t2dot*cos(t1-t2))*sin(t1-t2))
      /(len2*(m1+m2-m2*cos(t1-t2)*cos(t1-t2))) - t2ddot_damp ;

  ConsistentVector statedot(state.size());
  statedot[0] = state[1];
  statedot[1] = t1ddot;
  statedot[2] = state[3];
  statedot[3] = t2ddot;

  return statedot;
}


