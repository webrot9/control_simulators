
#include <iostream>
#include <cmath>
#include <fstream>

#include "../control_simulators/grid_world_taxi.h"

int main(int argc, char* argv[]) {
  float dt = 1;

  ConsistentVector init_state0(7);
  init_state0 << 0, 0, 0, 0, 0, 0, 0;

  ConsistentVector move_control(3);
  move_control << 0, 1, 0;
  ConsistentVector pick_control(3);
  pick_control << 0, 0, 1;
  ConsistentVector drop_control(3);
  drop_control << 0, 0, -1;

  ConsistentVector grid_sz(2);
  grid_sz << 3, 10;

  Eigen::MatrixXd occupied = Eigen::MatrixXd(1, 2);
  occupied << 1, 4;

  Eigen::MatrixXd walls = Eigen::MatrixXd::Zero(2, 4);
  walls.row(0) << 0, 1, 0, 2;
  walls.row(1) << 1, 0, 2, 0;

  Eigen::MatrixXd targets = Eigen::MatrixXd::Zero(1, 2);
  targets << 2, 5;

  Eigen::MatrixXd passengers = Eigen::MatrixXd::Zero(1, 2);
  passengers << 1, 7;

  GridWorldTaxi gwt(init_state0);
  gwt.setParam(GridWorldTaxi::GRID_SIZE, grid_sz);
  gwt.setParam(GridWorldTaxi::OCCUPIED_CELLS, occupied);
  gwt.setParam(GridWorldTaxi::WALLS, walls);
  gwt.setParam(GridWorldTaxi::PASSENGERS, passengers);
  gwt.setParam(GridWorldTaxi::AGENT_TARGET, targets);

  std::cout << "GWT:" << std::endl;
  std::cout << "State size: " << gwt.stateSize() << std::endl;
  std::cout << "Control size: " << gwt.controlSize() << std::endl;
  std::cout << "Simulable ID: " << gwt.id() << std::endl << std::endl;

  std::cout << "Pick Control: " << pick_control.transpose() << std::endl;
  gwt.step(dt, pick_control);
  std::cout << "State: [" << gwt.state().transpose()
            << "] at time " <<  gwt.time() << std::endl;

  gwt.vis(gwt.state());
  std::cout << "Move Control " << move_control.transpose()
            << " for " << dt << " steps" << std::endl;
  gwt.step(dt, move_control);
  gwt.vis(gwt.state());
  std::cout << "State: [" << gwt.state().transpose()
            << "] at time " <<  gwt.time() << std::endl;

  std::cout << "Drop Control: " << drop_control.transpose() << std::endl;
  gwt.step(dt, drop_control);
  std::cout << "State: [" << gwt.state().transpose()
            << "] at time " <<  gwt.time() << std::endl;

  std::cout << "Passengers: \n"
            << gwt.getParam(GridWorldTaxi::PASSENGERS) << std::endl;

  return 0;
}
