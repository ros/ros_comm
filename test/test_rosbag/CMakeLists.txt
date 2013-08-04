cmake_minimum_required(VERSION 2.8.3)

project(test_rosbag)
find_package(catkin REQUIRED COMPONENTS message_generation rosbag rosconsole roscpp rosgraph_msgs rostest rosunit topic_tools xmlrpcpp)

if(CATKIN_ENABLE_TESTING)
  find_package(Boost REQUIRED COMPONENTS regex program_options)
  find_package(BZip2 REQUIRED)

  add_subdirectory(bag_migration_tests)
endif()

catkin_package()

if(CATKIN_ENABLE_TESTING)
  include_directories(${catkin_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS})

  catkin_download_test_data(download_data_test_indexed_1.2.bag http://pr.willowgarage.com/data/rosbag/test_indexed_1.2.bag
    FILENAME test/test_indexed_1.2.bag MD5 523cc0deb66269c84be4a7c605ff5eb3)
  catkin_download_test_data(download_data_test_chatter_50hz.bag http://pr.willowgarage.com/data/rosbag/chatter_50hz.bag
    FILENAME test/chatter_50hz.bag MD5 00844db3729b1b5f34e767dc691bd531)
  catkin_download_test_data(download_data_test_future_version_2.1.bag http://pr.willowgarage.com/data/rosbag/test_future_version_2.1.bag
    FILENAME test/test_future_version_2.1.bag MD5 ddcb5e8711b16514eb640330ed93c332)

  catkin_add_nosetests(test/test_bag.py)

  configure_file(test/test_bag.cpp.in 
                 ${PROJECT_BINARY_DIR}/test/test_bag.cpp)
  catkin_add_gtest(test_bag ${PROJECT_BINARY_DIR}/test/test_bag.cpp)
  if(TARGET test_bag)
    target_link_libraries(test_bag ${catkin_LIBRARIES})
  endif()

  configure_file(test/play_play.test.in 
                 ${PROJECT_BINARY_DIR}/test/play_play.test)
  add_rostest(${PROJECT_BINARY_DIR}/test/play_play.test)
  configure_file(test/rosbag_play.test.in 
                 ${PROJECT_BINARY_DIR}/test/rosbag_play.test)
  add_rostest(${PROJECT_BINARY_DIR}/test/rosbag_play.test)

  add_rostest(test/latched_pub.test)
  add_rostest(test/latched_sub.test)
  add_rostest(test/record_two_publishers.test)
  add_rostest(test/record_one_publisher_two_topics.test)

  include_directories(${GTEST_INCLUDE_DIRS})
  add_executable(double_pub EXCLUDE_FROM_ALL test/double_pub.cpp)
  target_link_libraries(double_pub ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
  add_dependencies(double_pub rosgraph_msgs_genpy)
  if(TARGET tests)
    add_dependencies(tests double_pub)
  endif()
endif()
