
#include <iostream>
#include <cmath>
#include <fstream>

#include "../control_simulators/grid_world_taxi.h"

int main(int argc, char* argv[]) {
  ConsistentVector init_state0(6);
  init_state0 << 0, 0, 0, 0, 0, 0;

  float dt = 3;
  ConsistentVector control(2);
  control << 0, 1;
  ConsistentVector grid_sz(2);
  grid_sz << 3, 10;

  Eigen::MatrixXd occupied = Eigen::MatrixXd(1, 2);
  occupied << 1, 4;

  Eigen::MatrixXd walls = Eigen::MatrixXd::Zero(2, 4);
  walls.row(0) << 0, 0, 0, 1;
  walls.row(1) << 1, 0, 2, 0;

  Eigen::MatrixXd targets = Eigen::MatrixXd::Zero(1, 2);
  targets << 2, 5;

  GridWorldTaxi gwt(init_state0);
  gwt.setParam(GridWorld::GRID_SIZE, grid_sz);
  gwt.setParam(GridWorld::OCCUPIED_CELLS, occupied);
  gwt.setParam(GridWorld::WALLS, walls);
  gwt.setParam(GridWorld::AGENT_TARGET, targets);

  std::cout << "GWT:" << std::endl;
  std::cout << "State size: " << gwt.stateSize() << std::endl;
  std::cout << "Control size: " << gwt.controlSize() << std::endl;
  std::cout << "Simulable ID: " << gwt.id() << std::endl;
  std::cout << "State: [" << gwt.state().transpose()
            << "] at time " <<  gwt.time() << std::endl;
  std::cout << "Simulating for " << dt << " steps" << std::endl;
  std::cout << "Control: " << control.transpose() << std::endl;
  std::cout << "Before steps" << std::endl;
  gwt.vis(gwt.state());
  gwt.step(dt, control);
  std::cout << "After steps" << std::endl;
  std::cout << "State: [" << gwt.state().transpose()
            << "] at time " <<  gwt.time() << std::endl;
  gwt.vis(gwt.state());
  std::cout << "Rand state test: " << gwt.rndState().transpose() << std::endl;

  return 0;
}
