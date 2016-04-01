#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_

#include <string>
#include <vector>
#include <map>
#include <utility>
#include <memory>
#include <stdexcept>

#include "interfaces/simulable.h"


// simulator state 0 (everything ok) or -1 (busy/error)
class Simulator : public Simulable {
 public:
  typedef std::map<int, std::shared_ptr<ConsistentVector> > IdConsistentVectorMap;

  // constructors
  explicit Simulator() : Simulable(ConsistentVector(1)) {
    state_[0] = 0;
  }
  virtual ~Simulator() {}

  // useful functions
  virtual void restart();
  virtual void restart(int id, ConsistentVector init_state);
  // simulator has the ownership of the simulables_, returns the id of the created object
  template<typename T> int addSimulation(ConsistentVector init_state) {
    // can throw but the state is not modified
    std::shared_ptr<Simulable> pointer(new T(init_state));
    int new_state_size = pointer->stateSize();
    int new_control_size = pointer->controlSize();
    int id = pointer->id();

    // amortized constant complexity thanks to the hint on the position where to insert 
    // (C++98, position before to the position to insert)
    simulables_.insert(--simulables_.end(), std::make_pair(id, pointer));
    simulation_state_size_ += new_state_size;
    simulation_control_size_ += new_control_size;

    return id;
  }
  void removeSimulation(int id);
  virtual ConsistentVector buildControl(const IdConsistentVectorMap& control_map) const;
  virtual ConsistentVector buildState(const IdConsistentVectorMap& control_map) const;
  virtual ConsistentVector step(double dt, const ConsistentVector& control);
  virtual ConsistentVector transition(double& time, double dt, 
				      const ConsistentVector& state,
				      const ConsistentVector& control) const;

  // setters and getters
  int stateSize() const { return 1; }
  int controlSize() const { return 0; }
  int simulationStateSize() const { return simulation_state_size_; }
  int simulationControlSize() const { return simulation_control_size_; }
  // complexity log in the size of the map
  int simulationStateSize(int id) { return simulables_[id]->stateSize(); }
  // complexity log in the size of the map
  int simulationControlSize(int id) { return simulables_[id]->controlSize(); }
  // complexity log in the size of the map
  const ConsistentVector& simulationState(int id) { return simulables_[id]->state(); }
  int simulationSize() { return simulables_.size(); }

 private:
  typedef std::map<int, std::shared_ptr<Simulable> > sim_map;
  virtual ConsistentVector dynamics(float time, const ConsistentVector &state, const ConsistentVector &control) const { return ConsistentVector::Zero(0); }
  virtual void checkStateSize(const ConsistentVector& state) const;
  virtual void checkControlSize(const ConsistentVector &control) const;

  sim_map simulables_;

  int simulation_state_size_;
  int simulation_control_size_;
};

#endif  // _SIMULATOR_H_
