cmake_minimum_required(VERSION 2.8.3)
project(rosout)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp rosgraph_msgs)

find_package(Boost)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

include_directories(${Boost_INCLUDE_DIRS})

add_executable(rosout rosout.cpp)
target_link_libraries(rosout ${catkin_LIBRARIES})

install(TARGETS rosout
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
