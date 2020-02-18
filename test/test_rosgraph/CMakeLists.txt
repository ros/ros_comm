cmake_minimum_required(VERSION 3.0.2)

project(test_rosgraph)
find_package(catkin REQUIRED COMPONENTS rostest)
catkin_package()

if(CATKIN_ENABLE_TESTING)
  add_rostest(test/masterapi.test)
endif()
