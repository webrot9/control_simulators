#ifndef _DOUBLE_PENDULUM_H_
#define _DOUBLE_PENDULUM_H_

#include <control_simulators/interfaces/simulable.h>

// Double Pendulum State contains the following:
//  0,theta1
//  1,theta1dot
//  2,theta2
//  3,theta2dot
class DoublePendulum : public Simulable {
 public:
  enum ParamIndex {
    L1 = 0,
    L2,
    M1,
    M2,
    D1,
    D2,
    END
  };
  // constructors
  explicit DoublePendulum(const ConsistentVector& state) : Simulable(state) {
    checkStateSize(state);
    // These are members of the parent class that must be set by the child
    param_.resize(END);
    param_[L1] = 1.0;
    param_[L2] = 1.0;
    param_[M1] = 1.0;
    param_[M2] = 1.0;
    param_[D1] = 0.0;
    param_[D2] = 0.0;
    param_names_ = {{"l1",L1}, {"l2",L2},
                    {"m1",M1}, {"m2",M2},
                    {"d1",D1}, {"d2",D2}
                   };
  }
  virtual ~DoublePendulum() {}

  // useful functions
  virtual ConsistentVector dynamics(double time,
				    const ConsistentVector& state,
				    const ConsistentVector &control) const override;
  
  // setters and getters
  virtual int stateSize() const override { return 4; }
  virtual int controlSize() const override { return 1; }
};

#endif // _DOUBLE_PENDULUM_H_
