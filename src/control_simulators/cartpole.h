#ifndef _CARTPOLE_H_
#define _CARTPOLE_H_

#include <vector>
#include <stdexcept>

#include <control_simulators/interfaces/simulable.h>

// Cartpole state is: position, velocity, angle, angular velocity;
// for the angle, 0 is straight up, PI is straight down
class Cartpole : public Simulable {
 public:
  enum CartpoleParamType {
    CART_MASS = 0,
    POLE_MASS,
    POLE_LENGTH,
    CART_POLE_FRICTION,
    END
  };
  // constructors
  explicit Cartpole(const ConsistentVector& state) : Simulable(state) {
    checkStateSize(state);
    // These are members of the parent class that must be set by the child
    param_.resize(END);
    param_[CART_MASS] = 1.0;
    param_[POLE_MASS] = 0.1;
    param_[POLE_LENGTH] = 0.5;
    param_[CART_POLE_FRICTION] = 0.01;
    param_names_ = {{"cart_mass",CART_MASS}, {"pole_mass",POLE_MASS}, {"pole_length", POLE_LENGTH}, {"friction", CART_POLE_FRICTION}};
  }
  virtual ~Cartpole() {}

  // useful functions
  virtual ConsistentVector dynamics(double time,
				    const ConsistentVector& state,
				    const ConsistentVector &control) const;
  
  // setters and getters
  virtual int stateSize() const override { return 4; }
  virtual int controlSize() const override { return 1; }
};

#endif // _CARTPOLE_H_
