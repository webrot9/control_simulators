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

  static constexpr int dim = 6;
  static constexpr int control_dim = 2;

  // constructors
  explicit GridWorld(const ConsistentVector& state) : Simulable(state) {
    param_.resize(END);
    param_[NUM_AGENTS] = 1;
    checkStateSize(state);

    ConsistentVector gs = 4*ConsistentVector::Ones(2);
    setParam(GRID_SIZE, gs);
    setParam(OCCUPIED_CELLS, Eigen::MatrixXd::Zero(1, 1));
    setParam(WALLS, Eigen::MatrixXd::Zero(1, 1));
    setParam(AGENT_TARGET, Eigen::MatrixXd::Zero(1, 1));

    param_names_ = {
      {"grid_size", GRID_SIZE},
      {"occupied_cells", OCCUPIED_CELLS},
      {"walls", WALLS},
      {"num_agents", NUM_AGENTS},
      {"agent_target", AGENT_TARGET}
    };
    ConsistentVector zero_cmd = ConsistentVector::Zero(2*param_[NUM_AGENTS]);
    step(1.0, zero_cmd);
  }
  virtual ~GridWorld() {}

  // useful functions
  ConsistentVector directions(const ConsistentVector &pose);
  ConsistentVector step(double dt,
                        const ConsistentVector& control) override;
  void reset() override { reset(all_states_[0]); }
  void reset(const ConsistentVector& state) override;
  ConsistentVector rndState();
  void vis(const ConsistentVector &state);

  // setters and getters
  int numAgents() const { return param_[NUM_AGENTS]; }
  int agentDim() const { return dim; }
  int stateSize() const override { return param_[NUM_AGENTS]*dim; }
  int controlSize() const override { return param_[NUM_AGENTS]*control_dim; }

  void setParam(const std::string &name, Eigen::MatrixXd value) {
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
  ConsistentVector grid_size_;
  Eigen::MatrixXd occupied_cells_;
  Eigen::MatrixXd walls_;
  Eigen::MatrixXd agent_target_;
  std::random_device rd_;
};

#endif  // SRC_CONTROL_SIMULATORS_GRID_WORLD_H_
