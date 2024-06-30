cmake_minimum_required(VERSION 3.0.2)
project(optimizer)

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
  std_msgs
  visualization_msgs
  global_map
  Eigen3 REQUIRED
  PCL REQUIRED
  
)
find_package(osqp REQUIRED)
find_package(absl REQUIRED)
#find_package(IPOPT REQUIRED)
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
 LIBRARIES optimizer
 CATKIN_DEPENDS roscpp std_msgs visualization_msgs global_map
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
  src
)
file(GLOB_RECURSE MY_SOURCES "src/*.cpp" "src/tool/*.cc" "src/tool/math/*.cpp" "src/visualization/*.cpp" )
## Declare a C++ library
add_library(optimizer
${MY_SOURCES}
# src/nlp_manager.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/optimizer_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against


# link_directories(
#   "/usr/local/lib64"
#   "/usr/local/lib"
# )
target_link_libraries(optimizer
  ${catkin_LIBRARIES}
  # /usr/local/lib/libipopt.so
  # /usr/local/lib/libipopt.so.3
  libipopt.so
  libipopt.so.1
  
  libColPack.so.0

  libadolc.so.2

  libosqp.a
  absl::strings
)