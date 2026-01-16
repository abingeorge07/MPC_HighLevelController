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

    // Test SDF function
    // this->sdfBox2d_test();

    // Test distance to obstacle function
    // this->test_distanceToObstacle();
}

bool ObstacleLoader::loadObstacles() {
    // Load obstacle data from the specified file
    std::cout << "Loading obstacles from: " << filepath << std::endl;

    num_obstacles = -1;
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

    num_obstacles = obstacle_data.boxes.size();

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

// Signed Distance Function for 2D Box Obstacle
double ObstacleLoader::sdfBox2d(const Eigen::Vector2d& point, int idx) {
    const BoxObstacle& obs = obstacle_data.boxes[idx];
    // Transform point to obstacle's local frame
    Eigen::Vector2d local_point = point - obs.position.head<2>();
    Eigen::Matrix2d rot_2d = obs.rotation.topLeftCorner<2,2>();
    local_point = rot_2d.transpose() * local_point; // Inverse rotation
    Eigen::Vector2d half_size = obs.size.head<2>() * 0.5;   
    Eigen::Vector2d d = local_point.cwiseAbs() - half_size;

    // If both components of d are negative, point is inside the box
    double outside = std::max(d.x(), 0.0)*std::max(d.x(), 0.0) + std::max(d.y(), 0.0)*std::max(d.y(), 0.0); 
    outside = std::sqrt(outside); // If outside is zero, point is inside box

    // If outside, the first max will be positive, then min of positive and 0 is 0
    // If inside, both max are negative, will choose the one that is closer to the edge
    // then, min with 0 will keep it negative
    double inside = std::min(std::max(d.x(), d.y()), 0.0);
    return inside + outside;
}


// Test function for sdfBox2d
double ObstacleLoader::sdfBox2d_test(void) {
    BoxObstacle test_box;
    test_box.position = Eigen::Vector3d(0.0, 0.0, 0.0);
    test_box.size = Eigen::Vector3d(2.0, 2.0, 2.0);
    test_box.rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector2d test_point(0.85, 0.25);
    double distance = sdfBox2d(test_point, 0);
    std::cout << "SDF Distance to Box: " << distance << std::endl;
    return distance;
}

// Compute distance from a 3D point to the nearest obstacle
double ObstacleLoader::distanceToObstacle(const Eigen::Vector3d& point) {
    double min_distance = std::numeric_limits<double>::max();
    Eigen::Vector2d point_2d(point.x(), point.y());
    for (size_t box = 0; box < obstacle_data.boxes.size(); ++box) {
        double dist = sdfBox2d(point_2d, box);
        if (dist < min_distance) {
            min_distance = dist;
        }
    }
    return min_distance;
}

// Test function for distanceToObstacle
double ObstacleLoader::test_distanceToObstacle(void) {
    Eigen::Vector3d point(0.85, 0.25, 0.0); // Example test point
    double dist = distanceToObstacle(point);
    std::cout << "Distance to Nearest Obstacle: " << dist << std::endl;
    return dist;
}

void ObstacleLoader::closest_point_on_obb(const BoxObstacle& box, const Eigen::Vector2d& point,
                                     Eigen::Vector2d& closest_out) {
    // Transform point to obstacle's local frame
    Eigen::Vector2d local_point = point - box.position.head<2>();
    Eigen::Matrix2d rot_2d = box.rotation.topLeftCorner<2,2>();
    local_point = rot_2d.transpose() * local_point; // Inverse rotation

    Eigen::Vector2d half_size = box.size.head<2>() * 0.5;
    Eigen::Vector2d clamped;
    clamped.x() = std::max(-half_size.x(), std::min(half_size.x(), local_point.x()));
    clamped.y() = std::max(-half_size.y(), std::min(half_size.y(), local_point.y()));

    // Transform back to world frame
    closest_out = rot_2d * clamped + box.position.head<2>();
}

// Linearize box constraint at a given point
// void ObstacleLoader::linearizeBoxConstraint(int idx, const Eigen::Vector2d& point, double inflation,
//                                            Eigen::Vector2d& normal_out, double& rhs_out) {
//     const BoxObstacle& obs = obstacle_data.boxes[idx];

//     Eigen::Vector2d q;
    
//     closest_point_on_obb(obs, point, q);

//     Eigen::Vector2d diff = point - q;

//     double d = diff.norm();
    
//     if (d < 1e-8) {
//         // fallback: use vector from box center to pred
//         diff = point - obs.position.head<2>();
//         d = diff.norm();
//         if (d < 1e-8) {
//             normal_out = Eigen::Vector2d(1.0, 0.0);
//             rhs_out = normal_out.dot(q) + inflation;
//             return;
//         }
//     }
//     normal_out = diff / d;
//     rhs_out = normal_out.dot(q) + inflation;
// }

void ObstacleLoader::linearizeBoxConstraint(
    int idx,
    const Eigen::Vector2d& p,
    double inflation,
    Eigen::Vector2d& normal_out,
    double& rhs_out)
{
    double d;
    Eigen::Vector2d grad;

    boxSignedDistanceGradient(idx, p, inflation, d, grad);

    // Gradient of signed distance
    normal_out = grad;

    // First-order Taylor expansion:
    // d(p) + ∇d(p)^T (x - p) >= 0
    rhs_out = grad.dot(p) - d;
}


// Compute signed distance and its gradient for a box obstacle
void ObstacleLoader::boxSignedDistanceGradient(
    int idx,
    const Eigen::Vector2d& p,
    double inflation,
    double& dist_out,
    Eigen::Vector2d& grad_out)
{
    const BoxObstacle& obs = obstacle_data.boxes[idx];

    // Closest point on the (non-inflated) box
    Eigen::Vector2d q;
    closest_point_on_obb(obs, p, q);

    Eigen::Vector2d diff = p - q;
    double d = diff.norm();

    // Outside the box
    if (d > 1e-8) {
        dist_out = d - inflation;
        grad_out = diff / d;  // ∇d(p)
        return;
    }

    // Inside the box → fallback
    Eigen::Vector2d center = obs.position.head<2>();
    Eigen::Vector2d escape = p - center;
    double esc_norm = escape.norm();

    if (esc_norm < 1e-8) {
        grad_out = Eigen::Vector2d(1.0, 0.0);
        dist_out = -inflation;
        return;
    }

    grad_out = escape / esc_norm;
    dist_out = -inflation;
}
