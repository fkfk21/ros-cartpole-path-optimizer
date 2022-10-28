cmake_minimum_required(VERSION 3.0.2)
project(path_optimizer)

add_compile_options(-std=c++2a)

find_package(casadi REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  jsk_rviz_plugins
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_node main/test.cpp)
target_link_libraries(test_node
  ${catkin_LIBRARIES}
)

add_executable(casadi_test main/casadi_test.cpp)
target_link_libraries(casadi_test
  casadi
  ${catkin_LIBRARIES}
)

add_library(cartpole_solver src/DirectTranscriptSolver.cpp src/DirectCollocationSolver.cpp)
target_link_libraries(cartpole_solver casadi)

add_executable(cartpole src/test.cpp)
target_link_libraries(cartpole
  cartpole_solver
  ${catkin_LIBRARIES}
)