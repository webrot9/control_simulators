#include <iostream>
#include <cmath>
#include <fstream>

#include "../control_simulators/pendulum.h"

int main(int argc, char* argv[]) {
  ConsistentVector init_state(2);
  ConsistentVector init_state2(2);

  init_state[0] = M_PI;
  init_state[1] = 0.f;
  
  init_state2[0] = 0.5f*M_PI;
  init_state2[1] = 0.f;

  ConsistentVector control(1);
  control[0] = 0.f;

  Pendulum p(init_state2);
  p.setParam(Pendulum::DAMPING_COEFF, 0.3);

  const Eigen::MatrixXd cov(1e-2*Eigen::MatrixXd::Identity(p.stateSize(), p.stateSize()));
  p.setNoiseCovariance(cov);

  Pendulum p1(init_state);

  float dt = 0.05f;
  int timesteps = 1000;

  std::cout << "P:" << std::endl;
  std::cout << "State size: " << p.stateSize() << std::endl;
  std::cout << "Control size: " << p.controlSize() << std::endl;
  std::cout << "Simulable ID: " << p.id() << std::endl;
  std::cout << "State: [" << p.state().transpose() << "] at time " <<  p.time() << std::endl;
  std::cout << "Simulating for " << timesteps << " of " << dt << " seconds" << std::endl;

  for (int i = 0; i < timesteps; ++i) {
    p.step(dt, control);
  }

  std::cout << "State: [" << p.state().transpose() << "] at time " <<  p.time() << std::endl;

  std::ofstream file;
  file.open("./plot_damped.txt");

  std::vector<ConsistentVector> states = p.allStates();

  for (int i = 0; i < states.size(); ++i) {
    file << states[i].transpose() << std::endl;
  }

  file.close();
  std::cout << std::endl;

  std::cout << "P1:" << std::endl;
  std::cout << "State size: " << p1.stateSize() << std::endl;
  std::cout << "Control size: " << p1.controlSize() << std::endl;
  std::cout << "Simulable ID: " << p1.id() << std::endl;
  std::cout << "State: [" << p1.state().transpose() << "] at time " <<  p1.time() << std::endl;
  std::cout << "Simulating for " << timesteps << " of " << dt << " seconds" << std::endl;

  for (int i = 0; i < timesteps; ++i) {
    p1.step(dt, control);
  }

  std::cout << "State: [" << p1.state().transpose() << "] at time " <<  p1.time() << std::endl;
  std::cout << "Resetting" << std::endl;

  file.open("./plot_nondamped_top.txt");

  states = p1.allStates();

  for (int i = 0; i < states.size(); ++i) {
    file << states[i].transpose() << std::endl;
  }

  file.close();

  p1.reset(init_state2);

  std::cout << "State: [" << p1.state().transpose() << "] at time " <<  p1.time() << std::endl;
  std::cout << "Simulating for " << timesteps << " of " << dt << " seconds" << std::endl;

  for (int i = 0; i < timesteps; ++i) {
    p1.step(dt, control);
  }

  std::cout << "State: [" << p1.state().transpose() << "] at time " <<  p1.time() << std::endl;

  file.open("./plot_nondamped.txt");

  states = p1.allStates();

  for (int i = 0; i < states.size(); ++i) {
    file << states[i].transpose() << std::endl;
  }

  file.close();

  std::cout << "Initial state was : [" << p1.initialState().transpose() << "]" << std::endl;
  std::cout << "Resetting" << std::endl;
  p1.reset();
  
  std::cout << "State: [" << p1.state().transpose() << "] at time " <<  p1.time() << std::endl;

  return 0;
}
