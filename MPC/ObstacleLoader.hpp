#ifndef MPC_OBSTACLELOADER_HPP
#define MPC_OBSTACLELOADER_HPP
#include <iostream>
#include <fstream>
#include <string>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <vector>
#include <filesystem>

enum class ObstacleType {
    UNDEFINED,
    UNWALKABLE,
    WALKABLE,
};

struct BoxObstacle {
    Eigen::Vector3d position;     // Center of box
    Eigen::Vector3d size;         // Width, Depth, Height
    Eigen::Matrix3d rotation;     // Full 3D rotation matrix
};

// struct StairStep {
//     Eigen::Vector3d position;
//     Eigen::Vector3d size;
//     Eigen::Matrix3d rotation;
// };

struct Obstacle {
    ObstacleType type = ObstacleType::UNDEFINED;
    std::vector<BoxObstacle> boxes;
};

class ObstacleLoader {
public:
    ObstacleLoader(const std::string& filepath);
    bool loadObstacles();
    void printObstacles();
private:
    std::string filepath;
    ObstacleType obstacle_type;
    struct Obstacle obstacle_data;
};
#endif // MPC_OBSTACLELOADER_HPP