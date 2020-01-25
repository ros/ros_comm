cmake_minimum_required(VERSION 3.0.2)
project(xmlrpcpp)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

find_package(catkin REQUIRED COMPONENTS cpp_common rostime)

# The CFG_EXTRAS is only for compatibility, to be removed in Lunar.
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xmlrpcpp
  CATKIN_DEPENDS cpp_common rostime
  CFG_EXTRAS xmlrpcpp-extras.cmake
)

include_directories(
  include
  libb64/include
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

if(WIN32)
  add_definitions(-D_WINDOWS)
endif()

add_library(xmlrpcpp
  src/XmlRpcClient.cpp
  src/XmlRpcDispatch.cpp
  src/XmlRpcServer.cpp
  src/XmlRpcServerConnection.cpp
  src/XmlRpcServerMethod.cpp
  src/XmlRpcSocket.cpp
  src/XmlRpcSource.cpp
  src/XmlRpcUtil.cpp
  src/XmlRpcValue.cpp
  libb64/src/cdecode.c
  libb64/src/cencode.c
)

target_link_libraries(xmlrpcpp ${catkin_LIBRARIES})
if(WIN32)
  target_link_libraries(xmlrpcpp ws2_32)
endif()

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

install(TARGETS xmlrpcpp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
