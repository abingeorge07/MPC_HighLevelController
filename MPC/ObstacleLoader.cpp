#include "ObstacleLoader.hpp"

#ifndef MODEL_DIR
#define MODEL_DIR ""
#endif

using json = nlohmann::json;

ObstacleLoader::ObstacleLoader(const std::string& filepath) {
    // Initialize model directory
    std::filesystem::path model_dir = std::filesystem::path(MODEL_DIR);

    // Check filepath to ensure what the obstacle type is
    std::string include_tag = "obstacle";
    size_t pos = filepath.find(include_tag);

    if(pos != std::string::npos){
        obstacle_type = ObstacleType::UNWALKABLE;
    }
    else
        obstacle_type = ObstacleType::WALKABLE;

    
    // Prepend model directory to the filepath
    this->filepath = model_dir / filepath;

    this->loadObstacles();

    // Print loaded obstacles for verification
    // this->printObstacles();
}

bool ObstacleLoader::loadObstacles() {
    // Load obstacle data from the specified file
    std::cout << "Loading obstacles from: " << filepath << std::endl;

    // Open the JSON file
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "[WorldModel] Could not open JSON file: " << filepath << std::endl;
        return false;
    }

    // Parse the JSON content
    json j;
    try {
        file >> j;
    } catch (...) {
        std::cerr << "[WorldModel] JSON parsing failed.\n";
        return false;
    }

    // Update the obstacle type 
    obstacle_data.type = obstacle_type;
    for (auto& obs : j) {
        BoxObstacle obstacle;

        // Fill obstacle data
        obstacle.position = Eigen::Vector3d(
            (double)obs["position"][0],
            (double)obs["position"][1],
            (double)obs["position"][2]
        );

        obstacle.size = Eigen::Vector3d(
            (double)obs["size"][0],
            (double)obs["size"][1],
            (double)obs["size"][2]
        );

        // Assuming rotation is given as Euler angles in radians
        // We only care about yaw (rotation around Z-axis) for ground obstacles
        double yaw = (double)obs["euler"][2];
        Eigen::Matrix3d rotation;
        rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        obstacle.rotation = rotation;

        // Add to the list of obstacles
        obstacle_data.boxes.push_back(obstacle);

    }

    return true;
}

void ObstacleLoader::printObstacles() {
    std::cout << "Obstacle Type: " << (obstacle_data.type == ObstacleType::UNWALKABLE ? "UNWALKABLE" : "WALKABLE") << std::endl;
    for (const auto& box : obstacle_data.boxes) {
        std::cout << "Box Obstacle:" << std::endl;
        std::cout << "  Position: [" << box.position.transpose() << "]" << std::endl;
        std::cout << "  Size: [" << box.size.transpose() << "]" << std::endl;
        std::cout << "  Rotation Matrix:\n" << box.rotation << std::endl;
    }
}
