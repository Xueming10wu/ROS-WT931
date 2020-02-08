cmake_minimum_required(VERSION 2.8.3)
project(wt931)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  serial
)

catkin_package()

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(main_cpp
  include/wt931/ImuController.h
  src/ImuController.cpp
  include/wt931/CJY901.h
  src/CJY901.cpp
  src/main.cpp
)
add_dependencies(main_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main_cpp
  ${catkin_LIBRARIES}
)