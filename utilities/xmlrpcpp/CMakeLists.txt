cmake_minimum_required(VERSION 2.8.3)
project(xmlrpcpp)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

find_package(catkin REQUIRED COMPONENTS cpp_common)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xmlrpcpp
  CATKIN_DEPENDS cpp_common
)

include_directories(include ${catkin_INCLUDE_DIRS})
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
  )

if(WIN32)
  target_link_libraries(xmlrpcpp ws2_32)
endif()

# FIXME the headers should be in a package-specific subfolder but can not be for backward compatibility
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

install(TARGETS xmlrpcpp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
