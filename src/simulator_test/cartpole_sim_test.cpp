#include <iostream>
#include <cmath>
#include <fstream>

#include "../control_simulators/cartpole.h"

int main(int argc, char* argv[]) {
  ConsistentVector init_state(4);
  ConsistentVector init_state2(4);

  init_state[0] = M_PI;
  init_state[1] = 0.f;
  init_state[2] = 0.f;
  init_state[3] = 0.f;
  
  init_state2[0] = 0.f;
  init_state2[1] = 0.f;
  init_state2[2] = 0.5f*M_PI;
  init_state2[3] = 0.f;

  ConsistentVector control(1);
  control[0] = 0.f;

  Cartpole cp(init_state2);
  cp.setParam(Cartpole::CART_POLE_FRICTION, 0.3);
  Cartpole cp1(init_state);
  cp1.setParam(Cartpole::CART_POLE_FRICTION, 0.0);

  float dt = 0.05f;
  int timesteps = 1000;

  std::cout << "CP:" << std::endl;
  std::cout << "State size: " << cp.stateSize() << std::endl;
  std::cout << "Control size: " << cp.controlSize() << std::endl;
  std::cout << "Simulable ID: " << cp.id() << std::endl;
  std::cout << "State: [" << cp.state().transpose() << "] at time " <<  cp.time() << std::endl;
  std::cout << "Simulating for " << timesteps << " of " << dt << " seconds" << std::endl;

  for (int i = 0; i < timesteps; ++i) {
    cp.step(dt, control);
  }

  std::cout << "State: [" << cp.state().transpose() << "] at time " <<  cp.time() << std::endl;

  std::ofstream file;
  file.open("./plot_damped.txt");

  std::vector<ConsistentVector> states = cp.allStates();

  for (const auto &state : states) {
    file << state.transpose() << std::endl;
  }

  file.close();

  std::cout << std::endl;

  std::cout << "CP1:" << std::endl;
  std::cout << "State size: " << cp1.stateSize() << std::endl;
  std::cout << "Control size: " << cp1.controlSize() << std::endl;
  std::cout << "Simulable ID: " << cp1.id() << std::endl;
  std::cout << "State: [" << cp1.state().transpose() << "] at time " <<  cp1.time() << std::endl;
  std::cout << "Simulating for " << timesteps << " of " << dt << " seconds" << std::endl;

  for (int i = 0; i < timesteps; ++i) {
    cp1.step(dt, control);
  }

  std::cout << "State: [" << cp1.state().transpose() << "] at time " <<  cp1.time() << std::endl;

  file.open("./plot_nondamped_top.txt");

  states = cp1.allStates();
  
  for (const auto &state : states) {
    file << state.transpose() << std::endl;
  }

  file.close();

  std::cout << "Resetting" << std::endl;
  cp1.reset(init_state2);
  
  std::cout << "State: [" << cp1.state().transpose() << "] at time " <<  cp1.time() << std::endl;
  std::cout << "Simulating for " << timesteps << " of " << dt << " seconds" << std::endl;

  for (int i = 0; i < timesteps; ++i) {
    cp1.step(dt, control);
  }

  std::cout << "State: [" << cp1.state().transpose() << "] at time " <<  cp1.time() << std::endl;

  file.open("./plot_nondamped.txt");

  states = cp1.allStates();

  for (const auto &state : states) {
    file << state.transpose() << std::endl;
  }

  file.close();

  std::cout << "Initial state was : [" << cp1.initialState().transpose() << "]" << std::endl;
  std::cout << "Resetting" << std::endl;
  cp1.reset();
  
  std::cout << "State: [" << cp1.state().transpose() << "] at time " <<  cp1.time() << std::endl;

  return 0;
}
