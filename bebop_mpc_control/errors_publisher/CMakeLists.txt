cmake_minimum_required(VERSION 2.8.3)
project(errors_publisher)

find_package(catkin_simple REQUIRED)
catkin_simple()



############
# BINARIES #
############
cs_add_executable(errors_publisher
  src/errors_publisher.cpp
)



##########
# EXPORT #
##########

cs_install()
cs_export()
