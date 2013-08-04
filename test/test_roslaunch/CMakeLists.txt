cmake_minimum_required(VERSION 2.8.3)

project(test_roslaunch)
find_package(catkin REQUIRED COMPONENTS rostest)
catkin_package()

if(CATKIN_ENABLE_TESTING)
  add_rostest(test/roslaunch.test)
  add_rostest(test/env.test)
  add_rostest(test/params_basic.test)
endif()
