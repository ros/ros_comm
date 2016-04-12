cmake_minimum_required(VERSION 2.8.3)
project(rosconsole)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

find_package(catkin REQUIRED COMPONENTS cpp_common rostime rosunit)

find_package(Boost COMPONENTS regex system thread)

# select rosconsole backend
set(ROSCONSOLE_BACKEND "" CACHE STRING "Type of rosconsole backend, one of 'log4cxx', 'glog', 'print'")
set(rosconsole_backend_INCLUDE_DIRS)
set(rosconsole_backend_LIBRARIES)
if(ROSCONSOLE_BACKEND STREQUAL "" OR ROSCONSOLE_BACKEND STREQUAL "log4cxx")
  find_package(Log4cxx QUIET)
  if(NOT LOG4CXX_LIBRARIES)
    # backup plan, hope it is in the system path
    find_library(LOG4CXX_LIBRARIES log4cxx)
  endif()
  if(LOG4CXX_LIBRARIES)
    list(APPEND rosconsole_backend_INCLUDE_DIRS ${LOG4CXX_INCLUDE_DIRS})
    list(APPEND rosconsole_backend_LIBRARIES rosconsole_log4cxx rosconsole_backend_interface ${LOG4CXX_LIBRARIES})
    set(ROSCONSOLE_BACKEND "log4cxx")
  elseif(ROSCONSOLE_BACKEND STREQUAL "log4cxx")
    message(FATAL_ERROR "Couldn't find log4cxx library")
  endif()
endif()
if(ROSCONSOLE_BACKEND STREQUAL "" OR ROSCONSOLE_BACKEND STREQUAL "glog")
  find_package(PkgConfig)
  pkg_check_modules(GLOG QUIET libglog)
  if(GLOG_FOUND)
    list(APPEND rosconsole_backend_INCLUDE_DIRS ${GLOG_INCLUDE_DIRS})
    list(APPEND rosconsole_backend_LIBRARIES rosconsole_glog rosconsole_backend_interface ${GLOG_LIBRARIES})
    set(ROSCONSOLE_BACKEND "glog")
  elseif(ROSCONSOLE_BACKEND STREQUAL "glog")
    message(FATAL_ERROR "Couldn't find glog library")
  endif()
endif()
if(ROSCONSOLE_BACKEND STREQUAL "" OR ROSCONSOLE_BACKEND STREQUAL "print")
  list(APPEND rosconsole_backend_LIBRARIES rosconsole_print rosconsole_backend_interface)
  set(ROSCONSOLE_BACKEND "print")
endif()
message(STATUS "rosconsole backend: ${ROSCONSOLE_BACKEND}")

catkin_package(
  INCLUDE_DIRS include ${rosconsole_backend_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
  LIBRARIES rosconsole ${rosconsole_backend_LIBRARIES} ${Boost_LIBRARIES}
  CATKIN_DEPENDS cpp_common rostime
  CFG_EXTRAS rosconsole-extras.cmake
)

include(${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/rosconsole-extras.cmake)

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

include_directories(include ${catkin_INCLUDE_DIRS} ${rosconsole_backend_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(rosconsole_backend_interface src/rosconsole/rosconsole_backend.cpp)

add_library(rosconsole src/rosconsole/rosconsole.cpp)
target_link_libraries(rosconsole ${rosconsole_backend_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

if(ROSCONSOLE_BACKEND STREQUAL "log4cxx")
  add_library(rosconsole_log4cxx src/rosconsole/impl/rosconsole_log4cxx.cpp)
  target_link_libraries(rosconsole_log4cxx rosconsole_backend_interface ${LOG4CXX_LIBRARIES} ${Boost_LIBRARIES})

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/speed_test.cpp")
    add_executable(rosconsole_speed_test test/speed_test.cpp)
    target_link_libraries(rosconsole_speed_test rosconsole ${rosconsole_backend_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  endif()
elseif(ROSCONSOLE_BACKEND STREQUAL "glog")
  add_library(rosconsole_glog src/rosconsole/impl/rosconsole_glog.cpp)
  target_link_libraries(rosconsole_glog rosconsole_backend_interface ${GLOG_LIBRARIES})
elseif(ROSCONSOLE_BACKEND STREQUAL "print")
  add_library(rosconsole_print src/rosconsole/impl/rosconsole_print.cpp)
  target_link_libraries(rosconsole_print rosconsole_backend_interface)
else()
  message(FATAL_ERROR "Unknown rosconsole backend '${ROSCONSOLE_BACKEND}'")
endif()

if(CMAKE_HOST_UNIX)
  catkin_add_env_hooks(10.rosconsole SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
else()
  catkin_add_env_hooks(10.rosconsole SHELLS bat DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
endif()

install(TARGETS rosconsole rosconsole_${ROSCONSOLE_BACKEND} rosconsole_backend_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

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
