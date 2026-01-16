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
    double sdfBox2d(const Eigen::Vector2d& point, int idx);
    double sdfBox2d_test(void);
    double distanceToObstacle(const Eigen::Vector3d& point);
    double test_distanceToObstacle(void);
    int getNumObstacles() const { return num_obstacles; }
    void linearizeBoxConstraint(int idx, const Eigen::Vector2d& point, double inflation,
                                Eigen::Vector2d& normal_out, double& rhs_out);

    void closest_point_on_obb(const BoxObstacle& box, const Eigen::Vector2d& point,
                              Eigen::Vector2d& closest_out);
    void boxSignedDistanceGradient(int idx, const Eigen::Vector2d& p, double inflation,
                                   double& dist_out, Eigen::Vector2d& grad_out);         
    struct Obstacle obstacle_data;
private:
    std::string filepath;
    ObstacleType obstacle_type;
    int num_obstacles;
};
#endif // MPC_OBSTACLELOADER_HPP

