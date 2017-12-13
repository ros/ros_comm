cmake_minimum_required(VERSION 2.8.3)
project(test_secure_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_secure_ros
   CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories( ${catkin_INCLUDE_DIRS} )

add_executable( talker2 src/talker.cpp )
target_link_libraries( talker2 ${catkin_LIBRARIES} )

add_executable( listener2 src/listener.cpp )
target_link_libraries( listener2 ${catkin_LIBRARIES} )

add_executable( is_uri_match src/is_uri_match.cpp )
target_link_libraries( is_uri_match ${catkin_LIBRARIES} )

install(TARGETS talker2 listener2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python( PROGRAMS 
  nodes/listener.py 
  nodes/talker.py 
  nodes/rosxmlrpc.py 
  nodes/test_config.py 
  nodes/test_self.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
