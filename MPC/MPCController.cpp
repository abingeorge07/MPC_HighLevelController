/*
MPC Controller Implementation
*/

#include "MPCController.hpp"
#include "ObstacleLoader.hpp"

MPCController::MPCController() {
    // Load configuration from file
    std::cout << "MPCController initialized "<< std::endl;
}

void MPCController::start() {
    std::cout << "Starting MPC control loop..." << std::endl;
}
