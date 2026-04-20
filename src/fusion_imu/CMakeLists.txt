cmake_minimum_required(VERSION 3.0.2)
project(fusion_imu)

set(CMAKE_BUILD_TYPE "Debug")

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
add_subdirectory("third_party/Fusion")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(

  INCLUDE_DIRS include

  # LIBRARIES fusion_imu
  CATKIN_DEPENDS
  roscpp
  sensor_msgs

  # DEPENDS system_lib
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(

  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fusion_imu_node src/fusion_imu_node.cpp)
add_dependencies(fusion_imu_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fusion_imu_node
  ${catkin_LIBRARIES}
  Fusion
)
