cmake_minimum_required(VERSION 2.8.3)
project(test_topic_tools)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rostest
  std_msgs
  topic_tools
)

if(CATKIN_ENABLE_TESTING)
  add_message_files(
    NOINSTALL
    FILES
    TestStamped.msg)
  generate_messages(DEPENDENCIES std_msgs)
endif()

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

if(CATKIN_ENABLE_TESTING)
  include_directories(${catkin_INCLUDE_DIRS})

  add_rostest_gtest(${PROJECT_NAME}-test_shapeshifterstamped test/shapeshifterstamped.test test/test_shapeshifterstamped.cpp)
  target_link_libraries(${PROJECT_NAME}-test_shapeshifterstamped ${catkin_LIBRARIES})

  add_rostest(test/synchronized_throttle.test)
endif()
