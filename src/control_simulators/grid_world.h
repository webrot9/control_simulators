// *Copyright

#ifndef SRC_CONTROL_SIMULATORS_GRID_WORLD_H_
#define SRC_CONTROL_SIMULATORS_GRID_WORLD_H_

#include <set>
#include <map>
#include <vector>
#include <string>
#include <stdexcept>

#include <control_simulators/interfaces/simulable.h>

// GridWorld state is: position of agents and targets

class GridWorld : public Simulable {
 public:
  struct cvector_cmp {
    bool operator()(const ConsistentVector &a,
                    const ConsistentVector &b) {
      return a(0) <= b(0) && a(1) <= b(1);
    }
  };
  typedef std::set<ConsistentVector,
      GridWorld::cvector_cmp> ConsistentVectorSet;

  enum ParamIndex {
    NUM_AGENTS = 0,
    END,
    GRID_SIZE,
    OCCUPIED_CELLS,
    WALLS,
    AGENT_TARGET
  };

  // constructors
  explicit GridWorld(const ConsistentVector& state,
                     ConsistentVector grid_size = 4*ConsistentVector::Ones(2),
                     int num_agents = 1,
                     int dim = 6, int control_dim = 2)
      : Simulable(state), dim_(dim), control_dim_(control_dim) {
    param_.resize(END);
    param_[NUM_AGENTS] = num_agents;
    checkStateSize(state);

    setParam(GRID_SIZE, grid_size);
    setParam(OCCUPIED_CELLS, Eigen::MatrixXd::Zero(1, 1));
    setParam(WALLS, Eigen::MatrixXd::Zero(1, 1));
    setParam(AGENT_TARGET, Eigen::MatrixXd::Zero(num_agents, 1));

    param_names_ = {
      {"grid_size", GRID_SIZE},
      {"occupied_cells", OCCUPIED_CELLS},
      {"walls", WALLS},
      {"num_agents", NUM_AGENTS},
      {"agent_target", AGENT_TARGET}
    };

    for (int a = 0; a < num_agents; ++a) {
      state_.segment(a*dim_, grid_size.size()) =
          state.segment(a*dim_, grid_size.size());
      state_.segment(a*dim_ + grid_size.size(), dirsSize()) =
          directions(state.segment(a*dim_, grid_size.size()));
    }
  }
  virtual ~GridWorld() {}

  // useful functions
  ConsistentVector directions(const ConsistentVector &pose);
  void move(double dt, const ConsistentVector &control);
  ConsistentVector step(double dt,
                        const ConsistentVector& control) override;
  void reset() override { reset(all_states_[0]); }
  void reset(const ConsistentVector& state) override;
  ConsistentVector rndState();
  void vis(const ConsistentVector &state);

  // setters and getters
  int numAgents() const { return param_[NUM_AGENTS]; }
  int agentDim() const { return dim_; }
  int dirsSize() const { return 4; }
  virtual int stateSize() const { return param_[NUM_AGENTS]*dim_; }
  virtual int controlSize() const { return param_[NUM_AGENTS]*control_dim_; }

  virtual void setParam(const std::string &name, Eigen::MatrixXd value) {
    const size_t indx = param_names_.at(name);
    setParam(indx, value);
  }

  virtual void setParam(size_t index, Eigen::MatrixXd value) {
    if (index == GRID_SIZE) {
      grid_size_ = value;
    } else if (index == OCCUPIED_CELLS) {
      occupied_cells_ = value;
    } else if (index == WALLS) {
      walls_ = value;
    }  else if (index == AGENT_TARGET) {
      agent_target_ = value;
    }
  }

  Eigen::MatrixXd getParam(const std::string &name) {
    const size_t indx = param_names_.at(name);
    return getParam(indx);
  }

  Eigen::MatrixXd getParam(size_t index) {
    if (index == AGENT_TARGET) {
      return agent_target_;
    } else if (index == OCCUPIED_CELLS) {
      return occupied_cells_;
    } else if (index == WALLS) {
      return walls_;
    } else if (index == GRID_SIZE) {
      return grid_size_;
    }
    return Eigen::MatrixXd::Zero(1, 1);
  }

 protected:
  int dim_;
  int control_dim_;
  ConsistentVector grid_size_;
  Eigen::MatrixXd occupied_cells_;
  Eigen::MatrixXd walls_;
  Eigen::MatrixXd agent_target_;
  std::random_device rd_;
};

#endif  // SRC_CONTROL_SIMULATORS_GRID_WORLD_H_
