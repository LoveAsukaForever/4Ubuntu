cmake_minimum_required(VERSION 3.0.2)
project(my_planner)

# # Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions(-DTEMP)

find_package(catkin REQUIRED COMPONENTS
        nav_core
        pluginlib
        roscpp
        tf
        dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
        cfg/MyPlanner.cfg
)

find_package(OpenCV REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES my_planner
        CATKIN_DEPENDS
        nav_core
        pluginlib
        roscpp
        tf
        dynamic_reconfigure

        # DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

# # Declare a C++ library
add_library(my_planner
        src/my_planner.cpp
)
target_link_libraries(my_planner
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

# 指定目标之间的构建依赖关系，确保在构建目标 my_planner 之前，先构建其依赖的其他目标
add_dependencies(my_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_gdb src/test_gdb.cpp)
add_dependencies(test_gdb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_gdb
        ${catkin_LIBRARIES}
)

install(FILES my_planner_plugin.xml
        LIBRARY DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
