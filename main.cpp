/*

* Main entry point for the MPC High-Level Controller
*
* This file initializes the controller, loads the obstacle information,
* and starts the control loop.
*
*/
#include <iostream>
#include "MPCController.hpp"

void find_next_state(Eigen::Vector3d& x, const Eigen::Vector2d& u, double dt) {
    // Simple unicycle model
    double th = x[2];
    double v = u[0];
    double w = u[1];

    x[0] += dt * v * std::cos(th);
    x[1] += dt * v * std::sin(th);
    x[2] += dt * w;
}


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
    MPCController mpcController(obstacleFile);

    // Assume initial state and goal are set here
    Eigen::Vector3d x0(0.0, 0.0, 0.0); // Example initial state
    Eigen::Vector3d goal(5.0, 5.0, 0.0); // Example goal state

    // Num of iterations to run
    int num_iterations = 1000;

    for (int i = 0; i < num_iterations; ++i) {   
        // Set references
        mpcController.set_references(x0, goal);

        // Solve MPC to get initial control
        Eigen::Vector2d u0;
        if (!mpcController.solveMPC(u0)) {
            std::cerr << "Failed to compute initial control." << std::endl;
            return -1;
        }

        // Print current state and control
        std::cout << "Current state: [" << x0.transpose() << "], Applied control: [" << u0.transpose() << "]" << std::endl;

        // Here, you would apply u0 to the system and update x0 accordingly
        find_next_state(x0, u0, 0.1); // Placeholder function to update state [MAKE SURE THE DT IS THE SAME AS MPC DT]

    }

    return 0;
}