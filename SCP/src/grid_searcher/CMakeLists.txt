cmake_minimum_required(VERSION 3.0.2)
project(grid_searcher)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

#debug
# SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  global_map
  Eigen3 REQUIRED
  PCL REQUIRED  
)
find_package(absl REQUIRED)
find_package(osqp REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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
 LIBRARIES grid_searcher
 CATKIN_DEPENDS roscpp sensor_msgs std_msgs visualization_msgs geometry_msgs  global_map
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
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)
file(GLOB_RECURSE MY_SOURCES "src/*.cpp" "src/hybrid_a_star.cc" "src/tool/*.cc")

## Declare a C++ library
add_library(grid_searcher ${MY_SOURCES}
)
# link_directories(
#   "/usr/local/lib64"
#   "/usr/local/lib"
# )
## Specify libraries to link a library or executable target against
# 不知道为什么找不到需要链接的库。findpackage 也找到了啊
target_link_libraries(grid_searcher
  ${catkin_LIBRARIES}
  libosqp.a
  absl::strings
)