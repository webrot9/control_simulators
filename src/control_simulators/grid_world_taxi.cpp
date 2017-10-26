// *Copyright

#include "./grid_world_taxi.h"


ConsistentVector GridWorldTaxi::step(double dt,
                                     const ConsistentVector &control) {
  int directions = 4;

  move(dt, control);
  ConsistentVector result = state_;
  for (int a = 0; a < param_[NUM_AGENTS]; ++a) {
    if (control(a*control_dim_ + grid_size_.size()) == 1) {
      result(a*dim_ + grid_size_.size() + directions) = 1;
    } else if (control(a*control_dim_ + grid_size_.size()) == -1) {
      result(a*dim_ + grid_size_.size() + directions) = 0;
    }
  }
  state_ = result;
  all_states_.push_back(state_);
  all_controls_.push_back(control);

  return state_;
}
