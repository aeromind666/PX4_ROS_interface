cmake_minimum_required(VERSION 2.8.3)
project("game_combat")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    xmlrpcpp
    nav_msgs
    geometry_msgs
    geographic_msgs
    mavros_msgs
    sensor_msgs
)


find_package(Eigen3 REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIR}
    include
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp std_msgs
)

add_library(game_uav src/game_uav.cpp)
add_library(game_formation_controller src/game_formation_controller.cpp)

add_executable(center_node src/game_formation_controller_node.cpp)
target_link_libraries(center_node 
    game_formation_controller
    game_uav
    ${catkin_LIBRARIES} 
    ${Eigen_LIBRARIES}
)


