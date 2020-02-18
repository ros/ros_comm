cmake_minimum_required(VERSION 3.0.2)

project(test_rosbag_storage)

find_package(catkin REQUIRED COMPONENTS rosbag_storage std_msgs)

find_package(Boost REQUIRED)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

  catkin_add_gtest(bag_player src/bag_player.cpp)
  if(TARGET bag_player)
    target_link_libraries(bag_player ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(create_and_iterate_bag src/create_and_iterate_bag.cpp)
  if(TARGET create_and_iterate_bag)
    target_link_libraries(create_and_iterate_bag ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(swap_bags src/swap_bags.cpp)
  if(TARGET swap_bags)
    target_link_libraries(swap_bags ${catkin_LIBRARIES})
  endif()
endif()
