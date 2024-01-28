project(go2_rl_lcm)
cmake_minimum_required(VERSION 3.5)
SET(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)

find_package(lcm REQUIRED) #lcm
include_directories(lcm_types)

# lcm
add_executable(lcm_position_go2 unitree_sdk2_bin/lcm_position_go2.cpp)
target_link_libraries(lcm_position_go2 lcm)
add_executable(lcm_receive unitree_sdk2_bin/lcm_receive.cpp)
target_link_libraries(lcm_receive lcm)