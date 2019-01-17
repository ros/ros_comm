cmake_minimum_required(VERSION 2.8.3)
project(message_filters)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp rosconsole)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES message_filters
  CATKIN_DEPENDS roscpp rosconsole
)
catkin_python_setup()

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/connection.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

if(CATKIN_ENABLE_TESTING)
  # Ugly workaround for check_test_ran macro issue
  #add_subdirectory(test)
  find_package(catkin COMPONENTS rostest rosunit)

  include_directories(${GTEST_INCLUDE_DIRS})

  # ********** Tests **********
  catkin_add_gtest(${PROJECT_NAME}-msg_cache_unittest test/msg_cache_unittest.cpp)
  if(TARGET ${PROJECT_NAME}-msg_cache_unittest)
    target_link_libraries(${PROJECT_NAME}-msg_cache_unittest message_filters ${GTEST_LIBRARIES})
  endif()

  catkin_add_gtest(${PROJECT_NAME}-time_synchronizer_unittest test/time_synchronizer_unittest.cpp)
  if(TARGET ${PROJECT_NAME}-time_synchronizer_unittest)
    target_link_libraries(${PROJECT_NAME}-time_synchronizer_unittest message_filters ${GTEST_LIBRARIES})
  endif()

  catkin_add_gtest(${PROJECT_NAME}-test_synchronizer test/test_synchronizer.cpp)
  if(TARGET ${PROJECT_NAME}-test_synchronizer)
    target_link_libraries(${PROJECT_NAME}-test_synchronizer message_filters ${GTEST_LIBRARIES})
  endif()

  catkin_add_gtest(${PROJECT_NAME}-test_exact_time_policy test/test_exact_time_policy.cpp)
  if(TARGET ${PROJECT_NAME}-test_exact_time_policy)
    target_link_libraries(${PROJECT_NAME}-test_exact_time_policy message_filters ${GTEST_LIBRARIES})
  endif()

  catkin_add_gtest(${PROJECT_NAME}-test_approximate_time_policy test/test_approximate_time_policy.cpp)
  if(TARGET ${PROJECT_NAME}-test_approximate_time_policy)
    target_link_libraries(${PROJECT_NAME}-test_approximate_time_policy message_filters ${GTEST_LIBRARIES})
  endif()

  catkin_add_gtest(${PROJECT_NAME}-test_simple test/test_simple.cpp)
  if(TARGET ${PROJECT_NAME}-test_simple)
    target_link_libraries(${PROJECT_NAME}-test_simple message_filters ${GTEST_LIBRARIES})
  endif()

  catkin_add_gtest(${PROJECT_NAME}-test_chain test/test_chain.cpp)
  if(TARGET ${PROJECT_NAME}-test_chain)
    target_link_libraries(${PROJECT_NAME}-test_chain message_filters ${GTEST_LIBRARIES})
  endif()

  # Needs to be a rostest because it spins up a node, which blocks until it hears from the master (unfortunately)
  add_rostest_gtest(${PROJECT_NAME}-time_sequencer_unittest test/time_sequencer_unittest.xml test/time_sequencer_unittest.cpp)
  if(TARGET ${PROJECT_NAME}-time_sequencer_unittest)
    target_link_libraries(${PROJECT_NAME}-time_sequencer_unittest message_filters)
  endif()

  add_rostest_gtest(${PROJECT_NAME}-test_subscriber test/test_subscriber.xml test/test_subscriber.cpp)
  if(TARGET ${PROJECT_NAME}-test_subscriber)
    target_link_libraries(${PROJECT_NAME}-test_subscriber message_filters)
  endif()

  # Unit test of the approximate synchronizer
  catkin_add_nosetests(test/test_approxsync.py)
  catkin_add_nosetests(test/test_message_filters_cache.py)
endif()
