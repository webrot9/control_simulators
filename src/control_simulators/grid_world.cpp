// *Copyright

#include "./grid_world.h"

ConsistentVector GridWorld::rndState() {
  ConsistentVector rnd_state = ConsistentVector::Zero(stateSize());

  for (int a = 0; a < param_[NUM_AGENTS]; ++a) {
    ConsistentVector rnd_pose = ConsistentVector::Zero(grid_size_.size());
    std::mt19937 mt(rd_());

    bool valid_cell = true;
    do {
      valid_cell = true;
      for (int g = 0; g < grid_size_.size(); ++g) {
        std::uniform_int_distribution<> distr(0, grid_size_(g) - 1);
        rnd_pose(g) = distr(mt);
      }

      for (int ar = 0; ar < a; ++ar) {
        if ((rnd_pose - rnd_state.segment(
                ar*dim_, grid_size_.size())).cwiseAbs().sum() == 0) {
          valid_cell = false;
          break;
        }
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

    rnd_state.segment(a*dim_, grid_size_.size()) = rnd_pose;
    rnd_state.segment(a*dim_ + rnd_pose.size(),
                      dirsSize()) = directions(rnd_pose);
  }
  return rnd_state;
}

ConsistentVector GridWorld::directions(const ConsistentVector &pose) {
  ConsistentVector dirs = ConsistentVector::Zero(dirsSize());
  double dir_val = 1;

  std::vector<ConsistentVector> directions(
      dirsSize(), ConsistentVector::Zero(control_dim_));
  directions.at(0) << pose(0) - 1, pose(1);
  directions.at(1) << pose(0) + 1, pose(1);
  directions.at(2) << pose(0), pose(1) - 1;
  directions.at(3) << pose(0), pose(1) + 1;

  for (unsigned int d = 0; d < directions.size(); ++d) {
    // grid size
    if (directions.at(d)(0) < 0 || directions.at(d)(0) >= grid_size_(0)
        || directions.at(d)(1) < 0 || directions.at(d)(1) >= grid_size_(1)) {
      dirs(d) = -dir_val;
      continue;
    }


    // Walls
    for (int w = 0; (walls_.cols() != 1) && w < walls_.rows(); ++w) {
      if ((pose - walls_.row(w).head(grid_size_.size())
           .transpose()).cwiseAbs().sum() == 0) {
        if ((directions.at(d) - walls_.row(w).tail(grid_size_.size())
             .transpose()).cwiseAbs().sum() == 0) {
          dirs(d) = -dir_val;
          break;
        }
      }
    }
    if (dirs(d) == -dir_val) continue;

    // Blocks
    for (int o = 0;
         (occupied_cells_.cols() != 1) && o < occupied_cells_.rows(); ++o) {
      if ((directions.at(d)
           - occupied_cells_.row(o).transpose()).cwiseAbs().sum() == 0) {
        dirs(d) = -dir_val;
        break;
      }
    }
    if (dirs(d) == -dir_val) continue;

    // Agents
    for (int a = 0; (param_[NUM_AGENTS] > 1.0) &&
             a < param_[NUM_AGENTS]; ++a) {
      if ((state_.segment(a*dim_, grid_size_.size())
           - directions.at(d)).cwiseAbs().sum() == 0) {
        dirs(d) = -dir_val;
        break;
      }
    }
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

  for (int a = 0; a < param_[NUM_AGENTS]; ++a) {
    state_.segment(a*dim_ + grid_size_.size(), dirsSize()) =
          directions(state_.segment(a*dim_, grid_size_.size()));
  }
}

void GridWorld::move(double dt, const ConsistentVector &control) {
  checkControlSize(control);
  bool non_det = true;

  std::mt19937 mt(rd_());
  std::uniform_int_distribution<> nd_cmd(0, 100);
  std::uniform_int_distribution<> int_dist(-1, 1);

  int steps = static_cast<int>(dt);
  steps = std::max(1, steps);
  for (int s = 0; s < steps; ++s) {
    std::map<int, ConsistentVector> agent_to_next;

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

      int delta_command_x = control(a*control_dim_) + nd_cmd_x;
      if (delta_command_x > 1) delta_command_x = 1;
      else if (delta_command_x < -1) delta_command_x = -1;
      int delta_command_y = control(a*control_dim_ + 1) + nd_cmd_y;
      if (delta_command_y > 1) delta_command_y = 1;
      else if (delta_command_y < -1) delta_command_y = -1;

      ConsistentVector next(control_dim_);
      next << state_(a*dim_) + delta_command_x,
          state_(a*dim_ + 1) + delta_command_y;

      agent_to_next.insert(std::make_pair(a, next));
    }

    std::map<int, ConsistentVector>::iterator agent =
        agent_to_next.begin();
    for (; agent != agent_to_next.end(); ++agent) {
      std::map<int, ConsistentVector>::iterator collider =
          agent_to_next.begin();
      for (; collider != agent_to_next.end(); ++collider) {
        if (agent->first == collider->first) continue;
        if ((agent->second - collider->second).cwiseAbs().sum() == 0) {
          agent_to_next.erase(agent->first);
        }
      }
    }

    for (int a = 0; a < param_[NUM_AGENTS]; ++a) {
      if (agent_to_next.find(a) == agent_to_next.end()) {
        continue;
      }

      ConsistentVector next = agent_to_next.at(a);

      bool wall = false;
      for (int w = 0; (walls_.cols() != 1) && w < walls_.rows(); ++w) {
        bool s1 = (state_.segment(a*dim_, grid_size_.size())
                   - walls_.row(w)
                   .head(grid_size_.size()).transpose()).norm() == 0;
        bool s2 = (next.head(grid_size_.size())
                   - walls_.row(w)
                   .tail(grid_size_.size()).transpose()).norm() == 0;
        if (s1 && s2) {
          wall = true;
          break;
        }
      }
      if (wall) continue;

      // moving towards obstacles
      bool occupied = false;
      for (int o = 0;
           (occupied_cells_.cols() != 1) && o < occupied_cells_.rows(); ++o) {
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

      // moving in other agents
      bool skip = false;
      for (int ao = 0; ao < param_[NUM_AGENTS]; ++ao) {
        if (ao == a) continue;
        if ((next - state_.segment(ao*dim_, grid_size_.size()))
            .cwiseAbs().sum() == 0) {
          skip = true;
          break;
        }
      }
      if (skip) continue;

      state_.segment(a*dim_, grid_size_.size()) = next;
      state_.segment(a*dim_ + grid_size_.size(), dirsSize()) =
                       directions(state_.segment(a*dim_, grid_size_.size()));
    }
    // needed to update a-1 agents
    for (int a = 0; a < param_[NUM_AGENTS]; ++a) {
      state_.segment(a*dim_ + grid_size_.size(), dirsSize()) =
          directions(state_.segment(a*dim_, grid_size_.size()));
    }
  }
}


ConsistentVector GridWorld::step(double dt,
                                 const ConsistentVector &control) {
  move(dt, control);
  all_states_.push_back(state_);  // todo bug fix
  all_controls_.push_back(control);  // todo bug fix :: appends only last command
  return state_;
}

void GridWorld::vis(const ConsistentVector &state) {
  if (grid_size_.size() > 2) return;
  for (int r = 0; r < grid_size_(0); ++r) {
    for (int c = 0; c < grid_size_(1); ++c) {
      ConsistentVector cell(2);
      cell << r, c;
      int agent = -1;
      bool occupied = false;
      bool target = false;

      // agent
      for (int s = 0; s < param_[NUM_AGENTS]; ++s) {
        if ((state.segment(s*dim_, 2) - cell).cwiseAbs().sum() == 0) {
          agent = s;
          break;
        }
      }

      // occupied
      for (int o = 0;
           (occupied_cells_.cols() != 1) && o < occupied_cells_.rows(); ++o) {
        if ((occupied_cells_.row(o).transpose() - cell).cwiseAbs().sum() == 0) {
          occupied = true;
          break;
        }
      }

      // target
      for (unsigned int g = 0; g < agent_target_.rows(); ++g) {
        if ((agent_target_.row(g).transpose() - cell).cwiseAbs().sum() == 0) {
          target = true;
          break;
        }
      }

      // wall
      bool side_wall = false;
      bool bottom_wall = false;
      for (int w = 0; (walls_.cols() != 1) && w < walls_.rows(); ++w) {
        ConsistentVector wall_diff =
            walls_.row(w).head(2) - walls_.row(w).tail(2);
        if ((cell - walls_.row(w).head(2).transpose()).cwiseAbs().sum() == 0) {
          if (wall_diff(1) < 0) {
            side_wall = true;
          }
          if (wall_diff(0) < 0) {
            bottom_wall = true;
          }
          break;
        }
      }
      std::string side_separator = ":";
      if (side_wall) side_separator = "|";
      std::string bottom_separator = ".";
      if (bottom_wall) bottom_separator = "_";


      if (agent != -1 && target) {
        std::cout << bottom_separator
            + std::to_string(agent) + "g" + side_separator;
      } else if (occupied) {
        std::cout << bottom_separator + "@@" + side_separator;
      } else if (agent != -1) {
        std::cout << bottom_separator + std::to_string(agent)
            + bottom_separator + side_separator;
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
