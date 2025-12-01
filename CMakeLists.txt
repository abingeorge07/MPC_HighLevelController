cmake_minimum_required(VERSION 3.10)
project(MyMujocoProject)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-g")

# Get current directory
get_filename_component(CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)

# Add MPC as a subdirectory - it defines the `MPC` target
add_subdirectory(${CURRENT_DIR}/MPC)

# Add json directory
add_subdirectory(${CURRENT_DIR}/json)

# Create executable
add_executable(minimal_mpc main.cpp)
target_link_libraries(minimal_mpc PUBLIC MPC)
