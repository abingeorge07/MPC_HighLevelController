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

    for (auto& obs : j) {
        BoxObstacle obstacle;
        std::cout << "Position of the obstacle is: " << obs["position"] << std::endl;
    }

    return true;
}
