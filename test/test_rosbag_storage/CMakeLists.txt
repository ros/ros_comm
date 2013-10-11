cmake_minimum_required(VERSION 2.8.3)

project(test_rosbag_storage)

find_package(catkin REQUIRED COMPONENTS rosbag_storage std_msgs)

find_package(Boost REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(create_bag src/create_bag.cpp)
target_link_libraries(create_bag ${catkin_LIBRARIES})

add_executable(iterate_bag src/iterate_bag.cpp)
target_link_libraries(iterate_bag ${catkin_LIBRARIES})
