cmake_minimum_required(VERSION 2.8.3)

project(rosbag_storage)

find_package(console_bridge REQUIRED)
find_package(catkin REQUIRED COMPONENTS cpp_common roscpp_serialization roscpp_traits rostime roslz4)
find_package(Boost REQUIRED COMPONENTS date_time filesystem program_options regex)
find_package(BZip2 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rosbag_storage
  CATKIN_DEPENDS roslz4
  DEPENDS console_bridge Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${console_bridge_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${BZIP2_INCLUDE_DIR})
add_definitions(${BZIP2_DEFINITIONS})

add_library(rosbag_storage
  src/bag.cpp
  src/bag_player.cpp
  src/buffer.cpp
  src/bz2_stream.cpp
  src/lz4_stream.cpp
  src/chunked_file.cpp
  src/message_instance.cpp
  src/query.cpp
  src/stream.cpp
  src/view.cpp
  src/uncompressed_stream.cpp
)

target_link_libraries(rosbag_storage ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${BZIP2_LIBRARIES} ${console_bridge_LIBRARIES})

install(TARGETS rosbag_storage
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
