/*

* Main entry point for the MPC High-Level Controller
*
* This file initializes the controller, loads the obstacle information,
* and starts the control loop.
*
*/
#include <iostream>
#include "MPCController.hpp"
#include "ObstacleLoader.hpp"


int main(int argc, char** argv) {
    // // Check for configuration file argument
    // if (argc < 3) {
    //     std::cerr << "Usage: " << argv[0] << " <config_file> <obstacle_file>" << std::endl;
    //     return -1;
    // }

    // std::string configFile = argv[1];
    // std::string obstacleFile = argv[2];

    std::string obstacleFile = "Obstacle_Info/obstacle.json";

    // Initialize MPC Controller
    // MPCController mpcController;

    // Load obstacles
    ObstacleLoader obstacleLoader(obstacleFile);

    // Start the MPC control loop
    // mpcController.start();

    return 0;
}