#include "./grid_world.h"

ConsistentVector GridWorld::step(double dt,
                                 const ConsistentVector &control) {
  checkControlSize(control);
  ConsistentVector result = state_;

  std::mt19937 mt(rd_());
  std::uniform_int_distribution<> int_dist(-1, 1);

  int steps = static_cast<int>(dt);
  steps = std::max(1, steps);
  for (int s = 0; s < steps; ++s) {
    std::map<int, ConsistentVector> agent_to_next;
    std::map<ConsistentVector, int, GridWorld::cvector_cmp> next_to_agent;

    for (int a = 0; a < param_[NUM_AGENTS]; ++a) {
      int delta_command_x = control(a*dim) + 0*int_dist(mt);
      if (delta_command_x > 1) delta_command_x = 1;
      else if (delta_command_x < -1) delta_command_x = -1;
      int delta_command_y = control(a*dim + 1) + 0*int_dist(mt);
      if (delta_command_y > 1) delta_command_y = 1;
      else if (delta_command_y < -1) delta_command_y = -1;

      ConsistentVector next(2);
      next << state_(a*dim) + delta_command_x,
          state_(a*dim + 1) + delta_command_y;


      std::map<int, ConsistentVector>::iterator collider = agent_to_next.begin();
      bool skip_loop = false;
      for (; collider != agent_to_next.end(); ++collider) {
        if (collider->second(0) == next(0)
            && collider->second(1) == next(1)) {
          skip_loop = true;
          break;
        }

      }
      if (skip_loop) {
        continue;
      } else  {
        agent_to_next.insert(std::make_pair(a, next));
        next_to_agent.insert(std::make_pair(next, a));
      }
    }

    for (int a = 0; a < param_[NUM_AGENTS]; ++a) {
      if (agent_to_next.find(a) == agent_to_next.end()) {
        continue;
      }

      ConsistentVector next = agent_to_next.at(a);
      // std::cout << "Command: "  <<  next.transpose() << std::endl;
      // moving towards obstacles
      bool occupied = false;
      ConsistentVectorSet::iterator occupier;
      for (occupier = occupied_cells_.begin();
           occupier != occupied_cells_.end(); ++occupier) {
        if ((*occupier)(0) == next(0)  && (*occupier)(1) == next(1)) {
          occupied = true;
          break;
        }
      }
      if (occupied) continue;
      // if (occupied_cells_.find(next) != occupied_cells_.end()) {
      //   // todo, does not work in vis,
      //   // due to missing definition of ConsistentVector::operator= ?
      //   continue;
      // }

      // moving out of the gridworld
      if (next(0) >= grid_size_(0) || next(1) >= grid_size_(1)
          || next(0) < 0 || next(1) < 0) {
        continue;
      }

      result.segment(a*dim, 2) = next;
    }

    state_ = result;
    all_states_.push_back(state_);
    all_controls_.push_back(control);
  }

  return state_;
}


void GridWorld::vis(const ConsistentVector &state) {
  if (grid_size_.size() > 2) return;

  for (int r = 0; r < (grid_size_)(0); ++r) {
    for (int c = 0; c < (grid_size_)(1); ++c) {
      ConsistentVector cell(2);
      cell << r, c;
      bool agent = false;
      bool occupied = false;
      bool target = false;

      for (int s = 0; s < state.size(); s+=2) {
        if ((state.segment(s, 2) - cell).norm() == 0) {
          agent = true;
          break;
        }
      }

      ConsistentVectorSet::iterator occupier;
      for (occupier = occupied_cells_.begin();
           occupier != occupied_cells_.end(); ++occupier) {
        if ((*occupier)(0) == r && (*occupier)(1) == c) {
          occupied = true;
          break;
        }
      }

      for (unsigned int g = 0; g < agent_target_.size(); ++g) {
        if (agent_target_.at(g)(0) == r && agent_target_.at(g)(1) == c) {
          target = true;
          break;
        }
      }

      if (agent && target) {
        std::cout << "_Ag|";
      } else if (occupied) {
        std::cout << "@@@|";
      } else if (agent) {
        std::cout << "_A_|";
      } else if (target) {
        std::cout << "__g|";
      } else {
        std::cout << "___|";
      }
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}
