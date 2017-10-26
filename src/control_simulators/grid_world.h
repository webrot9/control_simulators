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
    bool operator()(const ConsistentVector &a, const ConsistentVector &b) {
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

  // constructors
  explicit GridWorld(const ConsistentVector& state) : Simulable(state) {
    param_.resize(END);
    param_[NUM_AGENTS] = 1;
    checkStateSize(state);
    // These are members of the parent class that must be set by the child
    ConsistentVector gs = ConsistentVector(2);
    gs << 4, 4;
    setParam(GRID_SIZE, gs);
    setParam(OCCUPIED_CELLS, ConsistentVectorSet());
    setParam(WALLS, Eigen::MatrixXd::Zero(1, 1));
    ConsistentVector init_targets = 0.5*grid_size_;
    setParam(AGENT_TARGET,
             std::vector<ConsistentVector>(param_[NUM_AGENTS], init_targets));
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
  int controlSize() const override { return param_[NUM_AGENTS]*(dim - 4); }

  template<typename T>
      void setParam(const std::string &name, T value) {
    const size_t indx = param_names_.at(name);
    setParam(indx, value);
  }

  /* void setParam(size_t index, ConsistentVector value) { */
  /*   if (index == GRID_SIZE) { */
  /*     grid_size_ = value; */
  /*   } */
  /* } */

  void setParam(size_t index, ConsistentVectorSet value) {
    if (index == OCCUPIED_CELLS) {
      occupied_cells_ = value;
    }
  }

  void setParam(size_t index, std::vector<ConsistentVector> value) {
    if (index == AGENT_TARGET) {
      agent_target_ = value;
    }
  }

  void setParam(size_t index, Eigen::MatrixXd value) {
    if (index == WALLS) {
      walls_ = value;
    } else if (index == GRID_SIZE) {
      grid_size_ = value;
    }
  }

  template<typename T>
      T& getParam(const std::string &name) {
    const size_t indx = param_names_.at(name);
    return getParam<T>(indx);
  }

  template<typename T>
      T& getParam(size_t index) {
    void* datum = nullptr;

    if (index == AGENT_TARGET) {
      datum = static_cast<void*>(&agent_target_);
    } else if (index == OCCUPIED_CELLS) {
      datum = static_cast<void*>(&occupied_cells_);
    } else if (index == WALLS) {
      datum = static_cast<void*>(&walls_);
    } else if (index == GRID_SIZE) {
      datum = static_cast<void*>(&grid_size_);
    }

    T* value = static_cast<T*>(datum);
    return *value;
  }

  ConsistentVector grid_size_;
  ConsistentVectorSet occupied_cells_;
  Eigen::MatrixXd walls_;
  std::vector<ConsistentVector> agent_target_;
  std::random_device rd_;
};

#endif  // SRC_CONTROL_SIMULATORS_GRID_WORLD_H_
