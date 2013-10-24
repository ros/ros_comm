cmake_minimum_required(VERSION 2.8.3)
project(rosconsole)

find_package(catkin REQUIRED COMPONENTS cpp_common rostime rosunit)

set(rosconsole_LIBRARIES)

find_package(Log4cxx QUIET)
if(NOT LOG4CXX_LIBRARIES)
  # backup plan, hope it is in the system path
  find_library(LOG4CXX_LIBRARIES log4cxx)
endif()
if(LOG4CXX_LIBRARIES)
  list(APPEND rosconsole_LIBRARIES rosconsole_log4cxx ${LOG4CXX_LIBRARIES})
else()
  #message(FATAL_ERROR "Couldn't find log4cxx library")
  list(APPEND rosconsole_LIBRARIES rosconsole_empty)
endif()

find_package(Boost COMPONENTS regex thread)

include(cmake/rosconsole-extras.cmake)

catkin_package(
  INCLUDE_DIRS include ${Boost_INCLUDE_DIRS} ${LOG4CXX_INCLUDE_DIRS}
  LIBRARIES rosconsole ${rosconsole_LIBRARIES} ${Boost_LIBRARIES} 
  CATKIN_DEPENDS cpp_common rostime
  CFG_EXTRAS rosconsole-extras.cmake
)

# See ticket: https://code.ros.org/trac/ros/ticket/3626
# On mac use g++-4.2
IF(${CMAKE_SYSTEM} MATCHES "Darwin-11.*")
  IF(EXISTS "/usr/bin/g++-4.2")
    set(CMAKE_CXX_COMPILER /usr/bin/g++-4.2)
  ELSE(EXISTS "/usr/bin/g++-4.2")
    # If there is no g++-4.2 use clang++
    set(CMAKE_CXX_COMPILER /usr/bin/clang++)
  ENDIF(EXISTS "/usr/bin/g++-4.2")
ENDIF(${CMAKE_SYSTEM} MATCHES "Darwin-11.*")

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
if(LOG4CXX_LIBRARIES)
  include_directories(${LOG4CXX_INCLUDE_DIRS})
endif()

add_library(rosconsole src/rosconsole/rosconsole.cpp)
target_link_libraries(rosconsole ${rosconsole_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

if(LOG4CXX_LIBRARIES)
  add_library(rosconsole_log4cxx src/rosconsole/impl/rosconsole_log4cxx.cpp)

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/speed_test.cpp")
    add_executable(rosconsole_speed_test test/speed_test.cpp)
    target_link_libraries(rosconsole_speed_test rosconsole ${rosconsole_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  endif()
else()
  add_library(rosconsole_empty src/rosconsole/impl/rosconsole_empty.cpp)
endif()

if(CMAKE_HOST_UNIX)
  catkin_add_env_hooks(10.rosconsole SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
else()
  catkin_add_env_hooks(10.rosconsole SHELLS bat DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
endif()

install(TARGETS rosconsole
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
if(LOG4CXX_LIBRARIES)
  install(TARGETS rosconsole_log4cxx
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
endif()

install(FILES config/rosconsole.config
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/ros/config)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-utest test/utest.cpp)
  if(TARGET ${PROJECT_NAME}-utest)
    target_link_libraries(${PROJECT_NAME}-utest ${PROJECT_NAME})
  endif()

  if(${CMAKE_SYSTEM_NAME} STREQUAL Linux)
    catkin_add_gtest(${PROJECT_NAME}-assertion_test test/assertion_test.cpp)
    if(TARGET ${PROJECT_NAME}-assertion_test)
      target_link_libraries(${PROJECT_NAME}-assertion_test ${PROJECT_NAME})
    endif()
  endif()

  catkin_add_gtest(${PROJECT_NAME}-thread_test test/thread_test.cpp)
  if(TARGET ${PROJECT_NAME}-thread_test)
    target_link_libraries(${PROJECT_NAME}-thread_test ${PROJECT_NAME})
  endif()
endif()
