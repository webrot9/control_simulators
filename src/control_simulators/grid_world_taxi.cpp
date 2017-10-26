// *Copyright

#include "./grid_world_taxi.h"

ConsistentVector GridWorldTaxi::step(double dt,
                                     const ConsistentVector &control) {
  if (control(0) > -1) GridWorld::step(dt, control);
  else {;
  }

  ConsistentVector result = state_;
  return  state_;
}
