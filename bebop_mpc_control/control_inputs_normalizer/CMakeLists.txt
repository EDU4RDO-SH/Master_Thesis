cmake_minimum_required(VERSION 3.0.2)
project(control_inputs_normalizer)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  roscpp
  std_msgs
)


## catkin specific configuration
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES control_inputs_normalizer
  CATKIN_DEPENDS geometry_msgs mav_msgs roscpp std_msgs
#  DEPENDS system_lib
)


## Specify additional locations of header files
include_directories(include
	include/control_inputs_normalizer
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(control_inputs_normalizer
	src/control_inputs_normalizer.cpp
)


## Add cmake target dependencies of the executable (to use messages from external packages)
add_dependencies(control_inputs_normalizer ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries(control_inputs_normalizer
	${catkin_LIBRARIES}	
)
