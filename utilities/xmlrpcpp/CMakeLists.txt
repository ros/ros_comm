cmake_minimum_required(VERSION 3.5)
project(xmlrpcpp)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

find_package(ament_cmake REQUIRED)

# The CFG_EXTRAS is only for compatibility, to be removed in Lunar.
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES xmlrpcpp
#  CATKIN_DEPENDS cpp_common
#  CFG_EXTRAS xmlrpcpp-extras.cmake
#)

include_directories(
  include
  libb64/include
)

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

if(WIN32)
  target_link_libraries(xmlrpcpp ws2_32)
endif()

ament_export_include_directories(include)
ament_export_libraries(xmlrpcpp)
ament_package()

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h")

install(TARGETS xmlrpcpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

