cmake_minimum_required(VERSION 3.0.2)
project(bebop_odometry_republisher)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  bebop_msgs
  nav_msgs
  roscpp
  std_msgs
)


## catkin specific configuration
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bebop_odometry_republisher
  CATKIN_DEPENDS bebop_msgs nav_msgs roscpp std_msgs
#  DEPENDS system_lib
)


## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(bebop_odometry
	src/bebop_odometry.cpp
)


## Add cmake target dependencies of the executable (to use messages from external packages)
add_dependencies(bebop_odometry 
	${catkin_EXPORTED_TARGETS}
)


## Specify libraries to link a library or executable target against
target_link_libraries(bebop_odometry
	${catkin_LIBRARIES}	
)
