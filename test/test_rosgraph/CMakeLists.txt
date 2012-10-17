cmake_minimum_required(VERSION 2.8.3)
project(test_rosgraph)
find_package(catkin REQUIRED COMPONENTS rostest)
catkin_package()

add_rostest(test/masterapi.test)
