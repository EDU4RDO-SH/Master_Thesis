cmake_minimum_required(VERSION 3.0.2)
project(mpc_traj_viz)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
		geometry_msgs
  	nav_msgs
  	roscpp
		trajectory_msgs
)


catkin_package(
	INCLUDE_DIRS include
	#  LIBRARIES mpc_traj_viz
	CATKIN_DEPENDS geometry_msgs nav_msgs roscpp trajectory_msgs
	#  DEPENDS system_lib
)


## Specify additional locations of header files
include_directories(include
		include/mpc_traj_viz
  	${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(mpc_traj_viz
	src/mpc_traj_viz.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(mpc_traj_viz
	${catkin_LIBRARIES}
)
