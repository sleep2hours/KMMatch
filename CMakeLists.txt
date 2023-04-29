cmake_minimum_required(VERSION 2.8.3)
project(group_transform)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  bspline_opt
  traj_utils
  message_generation
  cv_bridge
  ego_planner
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES group_transform
 #CATKIN_DEPENDS plan_env path_searching bspline_opt traj_utils 
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(group_transform
  src/group_transform.cpp
  src/KMMatch.cpp
  )
target_link_libraries(group_transform 
  ego_planner
  ${catkin_LIBRARIES}
  )
#add_dependencies(ego_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

