cmake_minimum_required(VERSION 2.8.3)
project(rostopic)
find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS scripts/rostopic
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  add_rostest(test/rostopic.test)
endif()
