#ifndef _PENDULUM_H_
#define _PENDULUM_H_

#include <vector>
#include <stdexcept>

#include <control_simulators/interfaces/simulable.h>


// Pendulum state is: angle, angular velocity;
// for the angle, PI is straight up, 0 is straight down
class Pendulum : public Simulable {
 public:
  enum ParamIndex {
    LENGTH = 0,
    DAMPING_COEFF,
    END
  };
  // constructors
  explicit Pendulum(const ConsistentVector& state) : Simulable(state) {
    checkStateSize(state);
    // These are members of the parent class that must be set by the child
    param_.resize(END);
    param_[LENGTH] = 1.0;
    param_[DAMPING_COEFF] = 0.0;
    param_names_ = {{"length",LENGTH}, {"damping",DAMPING_COEFF}};
  }
  virtual ~Pendulum() {}

  // useful functions
  virtual ConsistentVector dynamics(double time,
				    const ConsistentVector& state,
				    const ConsistentVector &control) const override;
  
  // setters and getters
  virtual int stateSize() const override { return 2; }
  virtual int controlSize() const override { return 1; }
};

#endif // _PENDULUM_H_
