cmake_minimum_required(VERSION 2.8.3)
project(bebop_linear_mpc)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_BUILD_TYPE Release)

if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64" AND NOT APPLE)
  add_definitions(-fPIC)
endif()

include_directories(lib)

#############
# LIBRARIES #
#############
cs_add_library(bebop_linear_mpc
  src/steady_state_calculation.cpp
	src/mpc_queue.cpp
	src/linear_mpc.cpp
)


cs_add_library(solver 
  lib/ldl.c
  lib/solver.c
  lib/matrix_support.c
  lib/util.c
  lib/variables_definition.c
)

target_link_libraries(bebop_linear_mpc solver)


############
# BINARIES #
############
cs_add_executable(bebop_linear_mpc_node
  src/linear_mpc_node.cpp
)

cs_add_executable(bebop_waypoint_publisher
  src/bebop_waypoint_publisher.cpp
)

cs_add_executable(bebop_attitude_publisher
  src/bebop_attitude_publisher.cpp
)



target_link_libraries(bebop_linear_mpc_node ${PROJECT_NAME})


##########
# EXPORT #
##########
cs_install()
cs_export()
    
