cmake_minimum_required(VERSION 3.0.2)
project(serial_hw)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  any_node
  any_worker
  geometry_msgs
  roscpp
  serial
  signal_handler
  std_msgs
  move_control
)

catkin_package(

  INCLUDE_DIRS include

  # LIBRARIES serial_hw
  CATKIN_DEPENDS
  any_node
  any_worker
  geometry_msgs
  roscpp
  serial
  signal_handler
  std_msgs
  move_control

  # DEPENDS system_lib
)

include_directories(

  include
  ${catkin_INCLUDE_DIRS}
)

# # Declare a C++ library
add_library(serial_hw
  src/SerialHW.cpp
)
target_link_libraries(serial_hw
  ${catkin_LIBRARIES}
)
add_library(ringbuffer
  src/Ringbuffer.cpp
)
target_link_libraries(ringbuffer
  ${catkin_LIBRARIES}
)

add_executable(serial_hw_node src/serial_hw_node.cpp)
add_dependencies(serial_hw_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(serial_hw_node
  ${catkin_LIBRARIES}
  serial_hw
  ringbuffer
)

catkin_install_python(PROGRAMS scripts/plot_comm_results.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
