cmake_minimum_required(VERSION 2.8.3)
project(rosservice)
find_package(catkin)
catkin_package()

catkin_python_setup()

install(PROGRAMS scripts/rosservice
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
