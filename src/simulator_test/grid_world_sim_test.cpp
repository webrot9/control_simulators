#include <iostream>
#include <cmath>
#include <fstream>

#include "../control_simulators/grid_world.h"

int main(int argc, char* argv[]) {
  ConsistentVector init_state0(6);
  init_state0 << 0, 0, 0, 0, 0, 0;
  ConsistentVector init_state1(6);
  init_state1 << 1, 0, 0, 0, 0, 0;

  ConsistentVector control(2);
  control << 1, 1;
  ConsistentVector grid_sz(2);
  grid_sz << 3, 10;

  GridWorld gw(init_state0);
  gw.setParam(GridWorld::GRID_SIZE, grid_sz);

  ConsistentVector ocell0(2);
  ocell0 << 1, 1;
  ConsistentVector ocell1(2);
  ocell1 << 0, 1;

  GridWorld::ConsistentVectorSet occupied_cells;
  occupied_cells.insert(ocell0);
  occupied_cells.insert(ocell1);
  gw.setParam(GridWorld::OCCUPIED_CELLS, occupied_cells);

  ConsistentVector target(2);
  target << 2, 5;
  std::vector<ConsistentVector> atargets =
      std::vector<ConsistentVector>(1, target);
  gw.setParam(GridWorld::AGENT_TARGET, atargets);

  float dt = 1;

  std::cout << "GW:" << std::endl;
  std::cout << "State size: " << gw.stateSize() << std::endl;
  std::cout << "Control size: " << gw.controlSize() << std::endl;
  std::cout << "Simulable ID: " << gw.id() << std::endl;
  std::cout << "State: [" << gw.state().transpose() << "] at time " <<  gw.time() << std::endl;
  std::cout << "Simulating for " << dt << " steps" << std::endl;
  gw.step(dt, control);
  std::cout << "State: [" << gw.state().transpose() << "] at time " <<  gw.time() << std::endl;
  gw.vis(gw.state());

  std::cout << "Rand state test: " << gw.rndState().transpose() << std::endl;

  // std::ofstream file;
  // file.open("./plot_damped.txt");
  // std::vector<ConsistentVector> states = gw.allStates();
  // for (const auto &state : states) {
  //   file << state.transpose() << std::endl;
  // }
  // file.close();
  // std::cout << std::endl;

  std::cout << "Resetting" << std::endl;
  gw.reset(init_state1);

  std::cout << "State: [" << gw.state().transpose() << "] at time " <<  gw.time() << std::endl;
  std::cout << "Simulating for " << dt << " steps" << std::endl;
  gw.step(dt, control);
  std::cout << "State: [" << gw.state().transpose() << "] at time " <<  gw.time() << std::endl;
  gw.vis(gw.state());

  // file.open("./plot_nondamped.txt");
  // states = gw.allStates();
  // for (const auto &state : states) {
  //   file << state.transpose() << std::endl;
  // }
  // file.close();

  std::cout << "Initial state was : [" << gw.initialState().transpose() << "]" << std::endl;
  std::cout << "Resetting" << std::endl;
  gw.reset();
  std::cout << "State: [" << gw.state().transpose() << "] at time " <<  gw.time() << std::endl;

  return 0;
}
