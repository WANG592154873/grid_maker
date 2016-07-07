cmake_minimum_required(VERSION 2.8.3)
project(grid_maker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  visualization_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES grid_maker
#  CATKIN_DEPENDS roscpp tf visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(drawGrip src/drawGrip.cpp)
target_link_libraries(drawGrip ${catkin_LIBRARIES})