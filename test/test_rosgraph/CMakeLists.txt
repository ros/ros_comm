cmake_minimum_required(VERSION 2.8.3)

if(NOT CATKIN_ENABLE_TESTING)
  return()
endif()

project(test_rosgraph)
find_package(catkin REQUIRED COMPONENTS rostest)
catkin_package()

add_rostest(test/masterapi.test)
