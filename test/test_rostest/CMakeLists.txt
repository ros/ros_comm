cmake_minimum_required(VERSION 2.8.3)

project(test_rostest)

find_package(catkin REQUIRED COMPONENTS rostest)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  add_rostest(test/publishtest.test)
endif()
