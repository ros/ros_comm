cmake_minimum_required(VERSION 2.8.3)

project(test_rosbag_storage)

find_package(catkin REQUIRED COMPONENTS rosbag_storage std_msgs)

find_package(Boost REQUIRED)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

  catkin_add_gtest(create_and_iterate_bag src/create_and_iterate_bag.cpp)
  if(TARGET create_and_iterate_bag)
    target_link_libraries(create_and_iterate_bag ${catkin_LIBRARIES})
  endif()
endif()
