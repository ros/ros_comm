cmake_minimum_required(VERSION 2.8.3)
project(test_rosparam)
find_package(catkin REQUIRED COMPONENTS rostest)
catkin_package()

add_rostest(test/rosparam.test)

catkin_add_nosetests(test)
