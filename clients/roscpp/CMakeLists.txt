cmake_minimum_required(VERSION 3.0.2)
project(roscpp)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

find_package(catkin REQUIRED COMPONENTS
  cpp_common message_generation rosconsole roscpp_serialization roscpp_traits rosgraph_msgs rostime std_msgs xmlrpcpp
)

catkin_package_xml()
# split version in parts and pass to extra file
string(REPLACE "." ";" roscpp_VERSION_LIST "${roscpp_VERSION}")
list(LENGTH roscpp_VERSION_LIST _count)
if(NOT _count EQUAL 3)
  message(FATAL_ERROR "roscpp version '${roscpp_VERSION}' does not match 'MAJOR.MINOR.PATCH' pattern")
endif()
list(GET roscpp_VERSION_LIST 0 roscpp_VERSION_MAJOR)
list(GET roscpp_VERSION_LIST 1 roscpp_VERSION_MINOR)
list(GET roscpp_VERSION_LIST 2 roscpp_VERSION_PATCH)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/ros/common.h.in ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/ros/common.h)

find_package(Boost REQUIRED COMPONENTS chrono filesystem system)

# Make sure we use CLOCK_MONOTONIC for the condition variable wait_for if not Apple.
if(NOT APPLE AND NOT WIN32)
  if(Boost_VERSION_MACRO LESS 106100)
    message(FATAL_ERROR "${PROJECT_NAME} requires Boost 1.61 or above.")
  endif()
  if(Boost_VERSION_MACRO LESS 106700)
    # CLOCK_MONOTONIC became the default in Boost 1.67:
    # https://github.com/boostorg/thread/commit/1e84b978b2bb0aae830cc14533dea3b7ddda5cde
    add_definitions(-DBOOST_THREAD_HAS_CONDATTR_SET_CLOCK_MONOTONIC)
  endif()
endif()

include_directories(include ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/ros ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
# this is needed for use within a bazel workspace. See #1548 for details.
include_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})

add_message_files(
  DIRECTORY msg
  FILES Logger.msg
)

add_service_files(
  DIRECTORY srv
  FILES Empty.srv GetLoggers.srv SetLoggerLevel.srv
)

generate_messages()

set(PTHREAD_LIB "")
find_package(Threads)
if(CMAKE_THREAD_LIBS_INIT)
  string(LENGTH ${CMAKE_THREAD_LIBS_INIT} _length)
  if(_length GREATER 2)
    string(SUBSTRING ${CMAKE_THREAD_LIBS_INIT} 0 2 _prefix)
    if(${_prefix} STREQUAL "-l")
      math(EXPR _rest_len "${_length} - 2")
      string(SUBSTRING ${CMAKE_THREAD_LIBS_INIT} 2 ${_rest_len} PTHREAD_LIB)
    endif()
  endif()
endif()

catkin_package(
  INCLUDE_DIRS include ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/ros
  LIBRARIES roscpp ${PTHREAD_LIB}
  CATKIN_DEPENDS cpp_common message_runtime rosconsole roscpp_serialization roscpp_traits rosgraph_msgs rostime std_msgs xmlrpcpp
  DEPENDS Boost
)

include(CheckIncludeFiles)
include(CheckFunctionExists)
include(CheckCXXSymbolExists)

# Not everybody has <ifaddrs.h> (e.g., embedded arm-linux)
CHECK_INCLUDE_FILES(ifaddrs.h HAVE_IFADDRS_H)
# Not everybody has trunc (e.g., Windows, embedded arm-linux)
CHECK_FUNCTION_EXISTS(trunc HAVE_TRUNC)
# Not everybody has epoll (e.g., Windows, BSD, embedded arm-linux) 
CHECK_CXX_SYMBOL_EXISTS(epoll_wait "sys/epoll.h" HAVE_EPOLL)

# Output test results to config.h
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/src/libros/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(roscpp 
  src/libros/master.cpp
  src/libros/network.cpp
  src/libros/subscriber.cpp
  src/libros/common.cpp
  src/libros/publisher_link.cpp
  src/libros/service_publication.cpp
  src/libros/connection.cpp
  src/libros/single_subscriber_publisher.cpp
  src/libros/param.cpp
  src/libros/service_server.cpp
  src/libros/wall_timer.cpp
  src/libros/xmlrpc_manager.cpp
  src/libros/publisher.cpp
  src/libros/timer.cpp
  src/libros/io.cpp
  src/libros/names.cpp
  src/libros/topic.cpp
  src/libros/topic_manager.cpp
  src/libros/poll_manager.cpp
  src/libros/publication.cpp
  src/libros/statistics.cpp
  src/libros/intraprocess_subscriber_link.cpp
  src/libros/intraprocess_publisher_link.cpp
  src/libros/callback_queue.cpp
  src/libros/service_server_link.cpp
  src/libros/service_client.cpp
  src/libros/node_handle.cpp
  src/libros/connection_manager.cpp
  src/libros/file_log.cpp
  src/libros/transport/transport.cpp
  src/libros/transport/transport_udp.cpp
  src/libros/transport/transport_tcp.cpp
  src/libros/subscriber_link.cpp
  src/libros/service_client_link.cpp
  src/libros/transport_publisher_link.cpp
  src/libros/transport_subscriber_link.cpp
  src/libros/service_manager.cpp
  src/libros/rosout_appender.cpp
  src/libros/init.cpp
  src/libros/subscription.cpp
  src/libros/subscription_queue.cpp
  src/libros/spinner.cpp
  src/libros/internal_timer_manager.cpp
  src/libros/message_deserializer.cpp
  src/libros/poll_set.cpp
  src/libros/service.cpp
  src/libros/this_node.cpp
  src/libros/steady_timer.cpp
  )

if(WIN32)
  add_definitions(-DNOGDI)
endif()

add_dependencies(roscpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(roscpp
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

if(WIN32)
  target_link_libraries(roscpp ws2_32)
endif()

#explicitly install library and includes
install(TARGETS roscpp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
install(FILES ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/ros/common.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/ros)
# install legacy infrastructure needed by rosbuild
install(FILES rosbuild/roscpp.cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rosbuild)
catkin_install_python(PROGRAMS
  rosbuild/scripts/genmsg_cpp.py
  rosbuild/scripts/gensrv_cpp.py
  rosbuild/scripts/msg_gen.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rosbuild/scripts)
