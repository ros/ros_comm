cmake_minimum_required(VERSION 2.8.3)

project(rosbag_storage)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

find_package(console_bridge REQUIRED)
find_package(catkin REQUIRED COMPONENTS cpp_common pluginlib roscpp_serialization roscpp_traits rostime roslz4)
find_package(Boost REQUIRED COMPONENTS date_time filesystem program_options regex)
find_package(BZip2 REQUIRED)

catkin_package(
  CFG_EXTRAS rosbag_storage-extras.cmake
  INCLUDE_DIRS include
  LIBRARIES rosbag_storage
  CATKIN_DEPENDS pluginlib roslz4
  DEPENDS console_bridge Boost
)

# Support large bags (>2GB) on 32-bit systems
add_definitions(-D_FILE_OFFSET_BITS=64)

include_directories(include ${catkin_INCLUDE_DIRS} ${console_bridge_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${BZIP2_INCLUDE_DIR})
add_definitions(${BZIP2_DEFINITIONS})

set(AES_ENCRYPT_SOURCE "")
set(AES_ENCRYPT_LIBRARIES "")
if(NOT WIN32)
  set(AES_ENCRYPT_SOURCE "src/aes_encryptor.cpp")
  set(AES_ENCRYPT_LIBRARIES "crypto" "gpgme")
endif()

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
target_link_libraries(rosbag_storage ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${BZIP2_LIBRARIES} ${console_bridge_LIBRARIES} ${AES_ENCRYPT_LIBRARIES})

install(TARGETS rosbag_storage
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(rosbag_default_encryption_plugins
  ${AES_ENCRYPT_SOURCE}
  src/no_encryptor.cpp
)

target_link_libraries(rosbag_default_encryption_plugins ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${AES_ENCRYPT_LIBRARIES})

install(TARGETS rosbag_default_encryption_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

if(NOT WIN32)
  install(FILES encryptor_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

  if(CATKIN_ENABLE_TESTING)
    find_package(rostest)

    catkin_add_gtest(test_aes_encryptor test/test_aes_encryptor.cpp
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
    if(TARGET test_aes_encryptor)
      target_link_libraries(test_aes_encryptor rosbag_storage ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    endif()
  endif()
endif()
