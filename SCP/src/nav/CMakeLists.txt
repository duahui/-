cmake_minimum_required(VERSION 3.0.2)
project(nav)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)


#debug
###########
## 在可执行程序及相应的功能包均必须设置成debug调试的模式，否则就在顶层catkin_make命令 或 顶层cmakelists 的时候全部设置成debug调试。还是顶层catkin_make的命令处好用，也不需要对cmakelists处理
## 在nav的debug模式中，其他功能包的cpp文件是不可见的，只有h文件可见，所以list命令就不能用，但break可以用，这里就是相当于设置了一个潜在的未来break点，这样后续在动态连接时就会自己加上断点 ##
###########
# SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  grid_searcher
  optimizer
  roscpp
  std_msgs
  visualization_msgs
  Eigen3 REQUIRED
  PCL REQUIRED
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES nav
 CATKIN_DEPENDS geometry_msgs grid_searcher optimizer roscpp std_msgs visualization_msgs
#  DEPENDS system_lib
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(nav src/nav.cpp src/planner.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(nav ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(nav
  ${catkin_LIBRARIES}
)