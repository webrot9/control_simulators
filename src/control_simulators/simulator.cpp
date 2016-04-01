#include "simulator.h"


void Simulator::restart() {
  for (sim_map::iterator it = simulables_.begin(); it != simulables_.end(); ++it) {
    it->second->reset();
  }

  reset();
}

void Simulator::restart(int id, ConsistentVector init_state) {
  // complexity log in the size of the map
  simulables_[id]->reset(init_state);
}

void Simulator::removeSimulation(int id) {
  // complexity log in the size of the map
  sim_map::iterator it = simulables_.find(id);

  if (it != simulables_.end()) {
    // linear complexity
    int sim_state_size = it->second->stateSize();
    int sim_control_size = it->second->controlSize();

    simulables_.erase(it);
    simulation_state_size_ -= sim_state_size;
    simulation_control_size_ -= sim_control_size;
  }
}

ConsistentVector Simulator::buildControl(const IdConsistentVectorMap& control_map) const {
  ConsistentVector control(simulation_control_size_);
  control.setZero();
  
  int idx = 0;
  // total complexity n log(m), with m < n (potentially m << n)
  // n number of simulables
  // m number of mapped controls
  for (sim_map::const_iterator it = simulables_.begin(); it != simulables_.end(); ++it) {
    IdConsistentVectorMap::const_iterator control_map_it = control_map.find(it->first);
    int control_size = it->second->controlSize();

    if (control_map_it != control_map.end()) {
      control.block(idx, 1, control_size, 1) = *control_map_it->second;
    }

    idx += control_size;
  }

  return control;
}

ConsistentVector Simulator::buildState(const IdConsistentVectorMap& state_map) const {
  if (state_map.size() != simulables_.size()) {
    throw std::runtime_error("State can be built only if all the simulations (ids) are mapped");
  }

  ConsistentVector state(simulation_state_size_);
  state.setZero();
  
  int idx = 0;
  // total complexity n log(n), with n number of simulables
  for (sim_map::const_iterator it = simulables_.begin(); it != simulables_.end(); ++it) {
    IdConsistentVectorMap::const_iterator state_map_it = state_map.find(it->first);
    int state_size = it->second->stateSize();

    if (state_map_it != state_map.end()) {
      state.block(idx, 1, state_size, 1) = *state_map_it->second;
    }

    idx += state_size;
  }

  return state;
}

ConsistentVector Simulator::step(double dt, const ConsistentVector& control) {
  state_[0] = -1;
  checkControlSize(control);

  int idx = 0;

  for (sim_map::iterator it = simulables_.begin(); it != simulables_.end(); ++it) {
    int control_size = it->second->controlSize();
    ConsistentVector sim_control = control.block(idx, 1, control_size, 1);
    it->second->step(dt, sim_control);

    idx += control_size;
  }

  time_ += dt;
  state_[0] = 0;

  return state_;
}

ConsistentVector Simulator::transition(double& time, double dt, 
				       const ConsistentVector& state,
				       const ConsistentVector& control) const {
  checkStateSize(state);
  checkControlSize(control);

  int state_idx = 0;
  int control_idx = 0;

  for (sim_map::const_iterator it = simulables_.begin(); it != simulables_.end(); ++it) {
    double sim_time = time;
    int state_size = it->second->stateSize();
    int control_size = it->second->controlSize();
    ConsistentVector sim_state = state.block(state_idx, 1, state_size, 1);
    ConsistentVector sim_control = control.block(control_idx, 1, control_size, 1);
    it->second->transition(sim_time, dt, sim_state, sim_control);

    state_idx += state_size;
    control_idx += control_size;
  }

  time += dt;

  return ConsistentVector::Zero(0);
}

void Simulator::checkStateSize(const ConsistentVector& state) const {
  if (state.size() != simulationStateSize()) {
    throw std::runtime_error("State size must correspond!");
  }
}

void Simulator::checkControlSize(const ConsistentVector &control) const {
  if (control.size() != simulationControlSize()) {
    throw std::runtime_error("Control size must correspond!");
  }
}
