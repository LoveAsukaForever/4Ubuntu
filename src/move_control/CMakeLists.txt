cmake_minimum_required(VERSION 3.0.2)
project(move_control)

set(CMAKE_BUILD_TYPE "Debug")

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  move_base
  roscpp
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  tf2
  move_base_msgs
  actionlib
  tf
  message_generation
  nav_msgs
  visualization_msgs
  signal_handler
  pcl_ros
  pcl_conversions
)

add_message_files(
  FILES
  ActionMsg.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include

  # LIBRARIES
  CATKIN_DEPENDS
  move_base
  roscpp
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  tf2
  move_base_msgs
  actionlib
  tf
  message_runtime
  nav_msgs
  visualization_msgs
  signal_handler
  pcl_ros
  pcl_conversions

  # DEPENDS system_lib
)

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(send_goal src/sending_goal/send_goal.cpp)

add_dependencies(send_goal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(send_goal
  ${catkin_LIBRARIES}
)

# # Declare a C++ library
add_library(badminton_robot_fsm
  src/BadmintonRobotFSM.cpp
)
target_link_libraries(badminton_robot_fsm
  ${catkin_LIBRARIES}
  readline
)
add_library(visualization
  src/Visualization.cpp
)
target_link_libraries(visualization
  ${catkin_LIBRARIES}
)

add_executable(tf_odom_to_baselink_node src/tf_odom_to_baselink_node.cpp)
add_dependencies(tf_odom_to_baselink_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_odom_to_baselink_node ${catkin_LIBRARIES})

add_executable(test_pub_odom src/test_pub_odom.cpp)
add_dependencies(test_pub_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_pub_odom ${catkin_LIBRARIES})

add_executable(robot_fsm_node src/robot_fsm_node.cpp)
add_dependencies(robot_fsm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_fsm_node
  ${catkin_LIBRARIES}
  badminton_robot_fsm
  visualization
)

add_executable(pointcloud2_filter_node src/pointcloud2_filter_node.cpp)
add_dependencies(pointcloud2_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud2_filter_node ${catkin_LIBRARIES})

add_executable(test_fsm_node src/test_fsm_node.cpp)
add_dependencies(test_fsm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_fsm_node ${catkin_LIBRARIES})