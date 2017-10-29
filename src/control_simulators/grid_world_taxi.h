// *Copyright

#ifndef SRC_CONTROL_SIMULATORS_GRID_WORLD_TAXI_H_
#define SRC_CONTROL_SIMULATORS_GRID_WORLD_TAXI_H_

#include <set>
#include <map>
#include <vector>
#include <string>
#include <stdexcept>

#include <control_simulators/interfaces/simulable.h>
#include "./grid_world.h"

class GridWorldTaxi : public GridWorld {
 public:
  enum ParamIndex {
    NUM_AGENTS = 0,
    END,
    GRID_SIZE,
    OCCUPIED_CELLS,
    WALLS,
    AGENT_TARGET,
    PASSENGERS
  };

  // constructors
  explicit GridWorldTaxi(const ConsistentVector& state,
                         const ConsistentVector& grid_size)
      : GridWorld(state, grid_size, 1, 7, 3) {
    setParam(PASSENGERS, Eigen::MatrixXd::Zero(1, 1));
    param_names_ = {
      {"grid_size", GRID_SIZE},
      {"occupied_cells", OCCUPIED_CELLS},
      {"walls", WALLS},
      {"num_agents", NUM_AGENTS},
      {"agent_target", AGENT_TARGET},
      {"passengers", PASSENGERS}
    };

    assignments_.insert(std::make_pair(0, 0));
  }
  virtual ~GridWorldTaxi() {}

  // not so-useful functions
  int agentDim() const { return dim_; }
  void setAssignments(std::map<int, int> a) { assignments_ = a; }
  std::map<int, int> assignments() { return assignments_; }
  int pickValue() const { return pick_value_; }
  virtual int stateSize() const { return param_[NUM_AGENTS]*dim_; }
  virtual int controlSize() const { return param_[NUM_AGENTS]*control_dim_; }

  // useful functions
  ConsistentVector step(double dt,
                        const ConsistentVector& control) override;

  void setParam(size_t index, Eigen::MatrixXd value) {
    GridWorld::setParam(index, value);
    if (index == PASSENGERS) {
      passengers_ = value;
    }
  }

  Eigen::MatrixXd getParam(size_t index) {
    Eigen::MatrixXd value = GridWorld::getParam(index);
    if (index == PASSENGERS) {
      return passengers_;
    }
    return value;
  }

 private:
  Eigen::MatrixXd passengers_;
  std::map<int, int> assignments_;
  static constexpr int pick_value_ = 30;
};

#endif  // SRC_CONTROL_SIMULATORS_GRID_WORLD_TAXI_H_
