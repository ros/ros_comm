cmake_minimum_required(VERSION 2.8.3)

project(test_shm_transport)

find_package(catkin REQUIRED COMPONENTS genmsg rostest std_msgs)

if(CATKIN_ENABLE_TESTING)
  generate_messages(DEPENDENCIES std_msgs)
endif()

catkin_package()

if(CATKIN_ENABLE_TESTING)
  include_directories(include ${catkin_INCLUDE_DIRS})

  add_executable(shm_talker test/shm_talker.cpp)
  target_link_libraries(shm_talker ${catkin_LIBRARIES} -lrt)
  add_dependencies(shm_talker test_shm_transport_generate_messages_cpp)

  add_executable(shm_listener test/shm_listener.cpp)
  target_link_libraries(shm_listener ${catkin_LIBRARIES} -lrt)
  add_dependencies(shm_listener test_shm_transport_generate_messages_cpp)
endif()

