// *Copyright

#include "./grid_world_taxi.h"


ConsistentVector GridWorldTaxi::step(double dt,
                                     const ConsistentVector &control) {
  int directions = 4;

  move(dt, control);
  ConsistentVector result = state_;
  for (int a = 0; a < param_[NUM_AGENTS]; ++a) {
    if (!assignments_.count(a)) continue;
    ConsistentVector agent = result.segment(a*dim_, grid_size_.size());
    ConsistentVector passenger = passengers_.row(assignments_.at(a));

    if ((agent - passenger).cwiseAbs().sum() == 0
        && control(a*control_dim_ + grid_size_.size()) == 1) {
      result(a*dim_ + grid_size_.size() + directions) = pick_value_;

    } else if (result(a*dim_ + grid_size_.size() + directions) == pick_value_
               && (control(a*control_dim_ + grid_size_.size()) == -1)) {
      result(a*dim_ + grid_size_.size() + directions) = 0;
    }

    if (result(a*dim_ + grid_size_.size() + directions) == pick_value_) {
      passengers_.row(assignments_.at(a)) =
          result.segment(a*dim_, grid_size_.size());
    }
  }

  state_ = result;
  all_states_.push_back(state_);
  all_controls_.push_back(control);
  return state_;
}
