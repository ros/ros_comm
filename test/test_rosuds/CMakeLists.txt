cmake_minimum_required(VERSION 2.8.3)
project(test_rosuds)

find_package(Boost REQUIRED COMPONENTS date_time thread)
find_package(catkin REQUIRED COMPONENTS message_generation rostime roscpp rosconsole roscpp_serialization)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_service_files(DIRECTORY srv FILES TwoInts.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

macro(test_rosuds_build T)
  add_executable(${T}_uds ${T}/${T}.cpp)
  target_link_libraries(${T}_uds ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endmacro()

foreach(dir
    talker
    listener
    listener_unreliable
    add_two_ints_client
    add_two_ints_server
    )
  test_rosuds_build(${dir})
endforeach()
