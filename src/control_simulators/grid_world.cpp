// *Copyright

#include "./grid_world.h"

ConsistentVector GridWorld::rndState() {
  ConsistentVector rnd_pose = ConsistentVector::Zero(grid_size_.size());
  std::mt19937 mt(rd_());

  bool valid_cell = true;
  do {
    valid_cell = true;
    for (int g = 0; g < grid_size_.size(); ++g) {
      std::uniform_int_distribution<> distr(0, grid_size_(g) - 1);
      rnd_pose(g) = distr(mt);
    }

    ConsistentVectorSet::iterator occupier;
    for (int o = 0; o < occupied_cells_.rows(); ++o) {
      int g = 0;
      for (; g < grid_size_.size(); ++g) {
        if (occupied_cells_(o, g) != rnd_pose(g)) break;
      }
      if (g == grid_size_.size()) {
        valid_cell = false;
        break;
      }
    }
  } while (!valid_cell);

  ConsistentVector rnd_state = ConsistentVector::Zero(stateSize());
  rnd_state.head(grid_size_.size()) = rnd_pose;
  ConsistentVector dirs = directions(rnd_pose);
  rnd_state.tail(dirs.size()) = dirs;
  return rnd_state;
}

ConsistentVector GridWorld::directions(const ConsistentVector &pose) {
  ConsistentVector dirs = ConsistentVector::Zero(4);
  double dir_val = 10;

  ConsistentVector upd(2);
  upd << pose(0) - 1, pose(1);
  ConsistentVector downd(2);
  downd << pose(0) + 1, pose(1);
  ConsistentVector leftd(2);
  leftd << pose(0), pose(1) - 1;
  ConsistentVector rightd(2);
  rightd << pose(0), pose(1) + 1;

  for (int w = 0; w < walls_.rows(); ++w) {
    if (pose(0) == walls_(w, 0) && pose(1) == walls_(w, 1)) {
      if (upd(0) == walls_(w, 2) && upd(1) == walls_(w, 3)) {
        dirs(0) = -dir_val;
      }
      if (downd(0) == walls_(w, 2) && downd(1) == walls_(w, 3)) {
        dirs(1) = -dir_val;
      }
      if (leftd(0) == walls_(w, 2) && leftd(1) == walls_(w, 3)) {
        dirs(2) = -dir_val;
      }
      if (rightd(0) == walls_(w, 2) && rightd(1) == walls_(w, 3)) {
        dirs(3) = -dir_val;
      }
    }
  }

  for (int o = 0; o < occupied_cells_.rows(); ++o) {
    if (upd(0) == occupied_cells_(o, 0)
        && upd(1) == occupied_cells_(o, 1)) {
      dirs(0) = -dir_val;
      continue;
    }

    if (downd(0) == occupied_cells_(o, 0)
        && downd(1) == occupied_cells_(o, 1)) {
      dirs(1) = -dir_val;
      continue;
    }

    if (leftd(0) == occupied_cells_(o, 0)
        && leftd(1) == occupied_cells_(o, 1)) {
      dirs(2) = -dir_val;
      continue;
    }

    if (rightd(0) == occupied_cells_(o, 0)
        && rightd(1) == occupied_cells_(o, 1)) {
      dirs(3) = -dir_val;
    }
  }

  if (upd(0) < 0
      || upd(0) >= grid_size_(0)
      || upd(1) < 0
      || upd(1) >= grid_size_(1)) {
    dirs(0) = -dir_val;
  }

  if (downd(0) < 0
      || downd(0) >= grid_size_(0)
      || downd(1) < 0
      || downd(1) >= grid_size_(1)) {
    dirs(1) = -dir_val;
  }

  if (leftd(0) < 0
      || leftd(0) >= grid_size_(0)
      || leftd(1) < 0
      || leftd(1) >= grid_size_(1)) {
    dirs(2) = -dir_val;
  }

  if (rightd(0) < 0
      || rightd(0) >= grid_size_(0)
      || rightd(1) < 0
      || rightd(1) >= grid_size_(1)) {
    dirs(3) = -dir_val;
  }
  return dirs;
}

void GridWorld::reset(const ConsistentVector& state) {
  checkStateSize(state);
  time_ = 0.0;
  state_ = state;
  // reallocation guaranteed
  std::vector<ConsistentVector>().swap(all_states_);
  std::vector<ConsistentVector>().swap(all_controls_);
  all_states_.push_back(state_);
  ConsistentVector zero_cmd =
      ConsistentVector::Zero(control_dim*param_[NUM_AGENTS]);
  step(1.0, zero_cmd);
}

ConsistentVector GridWorld::step(double dt,
                                 const ConsistentVector &control) {
  checkControlSize(control);
  ConsistentVector result = state_;
  bool non_det = true;

  std::mt19937 mt(rd_());
  std::uniform_int_distribution<> nd_cmd(0, 100);
  std::uniform_int_distribution<> int_dist(-1, 1);

  int steps = static_cast<int>(dt);
  steps = std::max(1, steps);
  for (int s = 0; s < steps; ++s) {
    std::map<int, ConsistentVector> agent_to_next;
    std::map<ConsistentVector, int, GridWorld::cvector_cmp> next_to_agent;

    for (int a = 0; a < param_[NUM_AGENTS]; ++a) {
      int nd_cmd_x = 0;
      int nd_cmd_y = 0;
      if (non_det && nd_cmd(mt) < 5) {
        if (nd_cmd(mt)%2 == 0) {
          nd_cmd_x = int_dist(mt);
        } else {
          nd_cmd_y = int_dist(mt);
        }
      }

      int delta_command_x = control(a*control_dim) + nd_cmd_x;
      if (delta_command_x > 1) delta_command_x = 1;
      else if (delta_command_x < -1) delta_command_x = -1;
      int delta_command_y = control(a*control_dim + 1) + nd_cmd_y;
      if (delta_command_y > 1) delta_command_y = 1;
      else if (delta_command_y < -1) delta_command_y = -1;

      ConsistentVector next(control_dim);
      next <<  state_(a*dim) + delta_command_x,
          state_(a*dim + 1) + delta_command_y;
      ConsistentVector directions = ConsistentVector::Zero(4);

      std::map<int, ConsistentVector>::iterator collider =
          agent_to_next.begin();
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

      bool wall = false;
      for (int w = 0; w < walls_.rows(); ++w) {
        bool s1 = (result.segment(a*dim, 2)
                   - walls_.row(w).head(2)).norm() == 0;
        bool s2 = (next
                   - walls_.row(w).tail(2)).norm() == 0;
        if (s1 && s2) {
          wall = true;
          break;
        }
      }
      if (wall) continue;

      // moving towards obstacles
      bool occupied = false;
      for (int o = 0; o < occupied_cells_.rows(); ++o) {
        if (occupied_cells_(o, 0) == next(0)
            && occupied_cells_(o, 1) == next(1)) {
          occupied = true;
          break;
        }
      }
      if (occupied) continue;

      // moving out of the gridworld
      if (next(0) >= grid_size_(0) || next(1) >= grid_size_(1)
          || next(0) < 0 || next(1) < 0) {
        continue;
      }
      result.segment(a*dim, 2) = next;
      result.segment(a*dim + 2, 4) = directions(next);
    }
    state_ = result;
    all_states_.push_back(state_);
    all_controls_.push_back(control);
  }

  return  state_;
}

void GridWorld::vis(const ConsistentVector &state) {
  if (grid_size_.size() > 2) return;
  for (int r = 0; r < grid_size_(0); ++r) {
    for (int c = 0; c < grid_size_(1); ++c) {
      ConsistentVector cell(2);
      cell << r, c;
      bool agent = false;
      bool occupied = false;
      bool target = false;

      // agent
      for (int s = 0; s < param_[NUM_AGENTS]; ++s) {
        if ((state.segment(s*dim, 2) - cell).norm() == 0) {
          agent = true;
          break;
        }
      }

      // occupied
      for (int o = 0; o < occupied_cells_.rows(); ++o) {
        if (occupied_cells_(o, 0) == r
            && occupied_cells_(o, 1) == c) {
          occupied = true;
          break;
        }
      }

      // target
      for (unsigned int g = 0; g < agent_target_.rows(); ++g) {
        if (agent_target_(g, 0) == r && agent_target_(g, 1) == c) {
          target = true;
          break;
        }
      }

      // wall
      bool side_wall = false;
      bool bottom_wall = false;
      for (int w = 0; w < walls_.rows(); ++w) {
        ConsistentVector wall_diff = walls_.row(w).head(2) - walls_.row(w).tail(2);
        if ((cell - walls_.row(w).head(2).transpose()).norm() == 0.0) {
          if (wall_diff(1) < 0) {
            side_wall = true;
          }
          if (wall_diff(0) < 0) {
            bottom_wall = true;
          }
        }
      }
      std::string side_separator = ":";
      if (side_wall) side_separator = "|";
      std::string bottom_separator = ".";
      if (bottom_wall) bottom_separator = "_";


      if (agent && target) {
        std::cout << bottom_separator + "Ag" + side_separator;
      } else if (occupied) {
        std::cout << bottom_separator + "@@" + side_separator;
      } else if (agent) {
        std::cout << bottom_separator + "A" + bottom_separator + side_separator;
      } else if (target) {
        std::cout << bottom_separator + bottom_separator + "g" + side_separator;
      } else {
        std::cout << bottom_separator + bottom_separator
            + bottom_separator + side_separator;
      }
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}
