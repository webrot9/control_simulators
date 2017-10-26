#include <control_simulators/interfaces/simulable.h>

#include <cmath>
#include <stdexcept>

int Simulable::simulable_counter_ = 0;

namespace {
static const double RK4_INTEGRATION_CONSTANT = 1.0/6.0;
}

void Simulable::reset(const ConsistentVector& state) {
  checkStateSize(state);
  time_ = 0.0;
  state_ = state;
  // reallocation guaranteed
  std::vector<ConsistentVector>().swap(all_states_);
  std::vector<ConsistentVector>().swap(all_controls_);
  all_states_.push_back(state_);
}

ConsistentVector Simulable::step(double dt, const ConsistentVector& control) {
  checkControlSize(control);
  ConsistentVector result = transition(time_, dt, state_, control);

  // Generate random noise
  const auto &cov = gaussian_noise_.getCovariance();
  bool zero_noise = cov.trace() <=  1e-12;
  if (!zero_noise) {
    const auto noise = gaussian_noise_.sample();
    result += noise;
  }

  state_ = result;
  all_states_.push_back(state_);
  all_controls_.push_back(control);

  return state_;
}

ConsistentVector Simulable::transition(double& time, double dt,
                                       const ConsistentVector& state,
                                       const ConsistentVector& control) const {
  checkStateSize(state);
  checkControlSize(control);
  ConsistentVector result;

  // integration dt to be considered based on passed in dt
  double integration_dt = dt*integration_frequency_;
  int num_steps = static_cast<int>(std::ceil(dt/integration_dt));

  if (integration_dt > min_integration_dt_) {
    // Since we are above the min integration point,
    // compute how many integration steps we should
    // take in order to be at least at the min dt or lower dt
    num_steps = static_cast<int>(std::ceil(dt/min_integration_dt_));
    integration_dt = dt/static_cast<double>(num_steps);
  }

  result = state;

  for (int i = 0; i < num_steps; ++i) {
    result = rk4(time, integration_dt, result, control);
  }

  return result;
}

ConsistentVector Simulable::rk4(double& time, double dt,
                                const ConsistentVector& state,
                                const ConsistentVector& control) const {
  // Formula from: http://mathworld.wolfram.com/Runge-KuttaMethod.html
  checkStateSize(state);
  checkControlSize(control);
  const ConsistentVector& k1 = dynamics(time, state, control);
  const ConsistentVector& k2 =
      dynamics(time + 0.5*dt, state + 0.5*dt*k1, control);
  const ConsistentVector& k3 =
      dynamics(time + 0.5*dt, state + 0.5*dt*k2, control);
  const ConsistentVector& k4 = dynamics(time + dt, state + dt*k3, control);

  time += dt;
  ConsistentVector result =
      state + RK4_INTEGRATION_CONSTANT*dt*(k1 + 2.0*(k2 + k3) + k4);

  return result;
}

void Simulable::checkStateSize(const ConsistentVector& state) const {
  if (state.size() != stateSize()) {
    throw std::runtime_error("State size must correspond. ");
  }
}

void Simulable::checkControlSize(const ConsistentVector &control) const {
  if (control.size() != controlSize()) {
    throw std::runtime_error("Control size must correspond.");
  }
}

std::vector<std::string> Simulable::paramNames() {
  std::vector<std::string> names(param_names_.size());
  size_t i = 0;
  for (const auto &item : param_names_) {
    names[i] = item.first;
    ++i;
  }
  return names;
}
