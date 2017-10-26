
#include <iostream>
#include <cmath>
#include <fstream>

#include "../control_simulators/grid_world.h"

int main(int argc, char* argv[]) {
  ConsistentVector init_state0(6);
  init_state0 << 0, 0, 0, 0, 0, 0;
  ConsistentVector init_state1(6);
  init_state1 << 1, 0, 0, 0, 0, 0;

  float dt = 3;
  ConsistentVector control(2);
  control << 0, 1;
  ConsistentVector grid_sz(2);
  grid_sz << 3, 10;

  Eigen::MatrixXd occupied = Eigen::MatrixXd(2, 2);
  occupied << 1, 4, 0, 3;

  Eigen::MatrixXd walls = Eigen::MatrixXd::Zero(2, 4);
  walls.row(0) << 0, 0, 0, 1;
  walls.row(1) << 1, 0, 2, 0;

  Eigen::MatrixXd target = Eigen::MatrixXd::Zero(1, 2);
  target << 2, 5;

  GridWorld gw(init_state0);
  gw.setParam(GridWorld::GRID_SIZE, grid_sz);
  gw.setParam(GridWorld::OCCUPIED_CELLS, occupied);
  gw.setParam(GridWorld::WALLS, walls);
  gw.setParam(GridWorld::AGENT_TARGET, target);

  std::cout << "GW:" << std::endl;
  std::cout << "State size: " << gw.stateSize() << std::endl;
  std::cout << "Control size: " << gw.controlSize() << std::endl;
  std::cout << "Simulable ID: " << gw.id() << std::endl;
  std::cout << "State: [" << gw.state().transpose() << "] at time " <<  gw.time() << std::endl;
  std::cout << "Simulating for " << dt << " steps" << std::endl;
  std::cout << "Before" << std::endl;
  gw.vis(gw.state());
  std::cout << "Control: " << control.transpose() << std::endl;
  gw.step(dt, control);
  std::cout << "State: [" << gw.state().transpose() << "] at time " <<  gw.time() << std::endl;
  std::cout << "After" << std::endl;
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
  std::cout << "Before" << std::endl;
  gw.vis(gw.state());
  std::cout << "Control: " << control.transpose() << std::endl;
  gw.step(dt, control);
  std::cout << "State: [" << gw.state().transpose() << "] at time " <<  gw.time() << std::endl;
  std::cout << "After" << std::endl;
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
