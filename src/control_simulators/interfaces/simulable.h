
#ifndef _SIMULABLE_H_
#define _SIMULABLE_H_


#include <control_simulators/utils/consistent_vector.h>
#include <control_simulators/utils/multivariate_gaussian.h>

#include <cmath>
#include <exception>
#include <map>
#include <memory>
#include <string>
#include <vector>

class Simulable {
 public:
  // constructors
  explicit Simulable(const ConsistentVector& init_state)
      : state_(init_state), gaussian_noise_(init_state.size()) {
    id_ = simulable_counter_++;
    time_ = 0.0;
    all_states_.push_back(init_state);
  }
  virtual ~Simulable() {}

  // useful functions
  virtual void reset() { reset(all_states_[0]); }
  virtual void reset(const ConsistentVector& state);
  virtual ConsistentVector step(double dt, const ConsistentVector& control);
  virtual ConsistentVector transition(double& time, double dt,
                                      const ConsistentVector& state,
                                      const ConsistentVector& control) const;

  // setters and getters for Gaussian noise
  const Eigen::MatrixXd& getNoiseCovariance() const {
    return gaussian_noise_.getCovariance();
  }
  const Eigen::VectorXd& getNoiseMean() const {
    return gaussian_noise_.getMean();
  }
  void setNoiseMean(const Eigen::VectorXd& mean) {
    if (mean.size() != stateSize()) {
      throw std::runtime_error("Noise mean size not consistent.");
    }

    gaussian_noise_.setMean(mean);
  }
  void setNoiseCovariance(const Eigen::MatrixXd& covariance) {
    if (covariance.rows() != covariance.cols()
        || covariance.rows() != stateSize()) {
      throw std::runtime_error("Noise covariance size not consistent.");
    }

    gaussian_noise_.setCovariance(covariance);
  }

  // setters and getters for state, time, controls
  int id() const { return id_; }
  double time() const { return time_; }
  const ConsistentVector& state() { return state_; }
  const ConsistentVector& initialState() { return all_states_[0]; }
  const std::vector<ConsistentVector>& allStates() { return all_states_; }
  const std::vector<ConsistentVector>& allControls() { return all_controls_; }
  virtual int stateSize() const {
    throw std::logic_error("State size not defined for Simulable class.");
    return 1;
  }
  virtual int controlSize() const {
    throw std::logic_error("Control size not defined for Simulable class.");
    return 1;
  }

  // Get and set parameters of the system using
  // size_t indexing into parameters array (length of
  // parameters array is dependent on the specific dyanmical system)
  void setParam(size_t index, double value) {
    param_.at(index) = value;
  }
  double getParam(size_t index) const {
    return param_.at(index);
  }
  // Get and set parameters of the system using a string for the lookup
  double getParam(const std::string &name) const {
    const size_t indx = param_names_.at(name);
    return param_.at(indx);
  }
  void setParam(const std::string &name, double value) {
    const size_t indx = param_names_.at(name);
    param_.at(indx) = value;
  }
  // Get a list of the parameter names as strings
  // for setting/getting parameter values
  std::vector<std::string> paramNames();

 protected:
  static int simulable_counter_;
  static constexpr double integration_frequency_ = 0.1;
  static constexpr double min_integration_dt_ = 1e-4;

  // DO NOT TOUCH THIS VALUE, NEEDED FOR RK4 (OPTIMIZATION PURPOSES)
  virtual ConsistentVector rk4(double& time, double dt,
                               const ConsistentVector& state,
                               const ConsistentVector& control) const;
  virtual ConsistentVector dynamics(double time,
                                    const ConsistentVector& state,
                                    const ConsistentVector &control) const {
      throw std::logic_error("Dynamics not defined for Simulable class.");
      return ConsistentVector(1);
  }
  virtual void checkStateSize(const ConsistentVector& state) const;
  virtual void checkControlSize(const ConsistentVector &control) const;

  std::vector<double> param_;
  std::map<std::string, int> param_names_ = {};

  int id_;
  double time_;
  ConsistentVector state_;
  std::vector<ConsistentVector> all_states_;
  std::vector<ConsistentVector> all_controls_;

  MultivariateGaussian<double> gaussian_noise_;
};

#endif  // _SIMULABLE_H_
