cmake_minimum_required(VERSION 3.0.2)
project(rosservice)
find_package(catkin)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS scripts/rosservice
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
