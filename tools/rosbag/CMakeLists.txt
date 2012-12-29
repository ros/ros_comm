cmake_minimum_required(VERSION 2.8.3)
project(rosbag)
find_package(catkin REQUIRED COMPONENTS rosconsole roscpp topic_tools xmlrpcpp)
find_package(Boost REQUIRED COMPONENTS regex program_options)
find_package(BZip2 REQUIRED)

catkin_python_setup()

# Support large bags (>2GB) on 32-bit systems
add_definitions(-D_FILE_OFFSET_BITS=64)

include_directories(include ${Boost_INCLUDE_DIR} ${BZIP2_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_definitions(${BZIP2_DEFINITIONS})

catkin_package(
  LIBRARIES rosbag
  INCLUDE_DIRS include
  CATKIN_DEPENDS rosconsole roscpp topic_tools xmlrpcpp)

add_library(rosbag
  src/bag.cpp
  src/buffer.cpp
  src/bz2_stream.cpp
  src/chunked_file.cpp
  src/message_instance.cpp
  src/player.cpp
  src/query.cpp
  src/recorder.cpp
  src/stream.cpp
  src/time_translator.cpp
  src/uncompressed_stream.cpp
  src/view.cpp)
target_link_libraries(rosbag ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${BZIP2_LIBRARIES})

add_executable(record src/record.cpp)
target_link_libraries(record rosbag)

add_executable(play src/play.cpp)
target_link_libraries(play rosbag)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
install(TARGETS rosbag record play
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS
  scripts/bag2png.py
  scripts/bagsort.py
  scripts/fastrebag.py
  scripts/fixbag.py
  scripts/fixbag_batch.py
  scripts/fix_md5sums.py
  scripts/fix_moved_messages.py
  scripts/fix_msg_defs.py
  scripts/makerule.py
  scripts/savemsg.py
  scripts/topic_renamer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
