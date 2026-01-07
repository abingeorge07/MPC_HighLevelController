
# This CMakeLists is intended to be added from the top-level project via
# Add Eigen3 include path (adjust if necessary)
include_directories(/usr/include/eigen3)

# Define MPC library target
add_library(MPC
    MPCController.cpp
    ObstacleLoader.cpp
)

# Public include directory so consumers can include headers like <MPC/MPCController.hpp>
target_include_directories(MPC
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../json/include
)

# Link Eigen3
find_package(Eigen3 3.3 REQUIRED)
target_link_libraries(MPC PUBLIC Eigen3::Eigen)

# Link yaml-cpp
find_package(yaml-cpp REQUIRED)
target_link_libraries(MPC PUBLIC yaml-cpp)


# Link OSQP and OSQP-Eigen
find_package(OsqpEigen REQUIRED)
#
# include_directories(${OSQP_EIGEN_INCLUDE_DIRS})
target_link_libraries(MPC PUBLIC OsqpEigen::OsqpEigen)

# Require C++17 for this target
target_compile_features(MPC PUBLIC cxx_std_17)

# Target include directories
target_compile_definitions(MPC PUBLIC MODEL_DIR="${CURRENT_DIR}/")