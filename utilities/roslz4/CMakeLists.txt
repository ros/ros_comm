cmake_minimum_required(VERSION 2.8.3)

project(roslz4)

find_package(catkin REQUIRED)

find_path(lz4_INCLUDE_DIRS NAMES lz4.h)
if (NOT lz4_INCLUDE_DIRS)
  message(FATAL_ERROR "lz4 includes not found")
endif()

find_library(lz4_LIBRARIES NAMES lz4)
if (NOT lz4_LIBRARIES)
  message(FATAL_ERROR "lz4 library not found")
endif()

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roslz4
  DEPENDS lz4)

include_directories(include ${lz4_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_library(roslz4 src/lz4s.c src/xxhash.c)
target_link_libraries(roslz4 ${lz4_LIBRARIES} ${catkin_LIBRARIES})

# Python bindings
set(Python_ADDITIONAL_VERSIONS "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")
set(PythonLibs_FIND_VERSION "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})

add_library(roslz4_py src/_roslz4module.c)
set_target_properties(
  roslz4_py PROPERTIES OUTPUT_NAME roslz4 PREFIX "_" SUFFIX ".so"
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${PYTHON_INSTALL_DIR}/roslz4)
target_link_libraries(roslz4_py roslz4 ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})

install(TARGETS roslz4
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS roslz4_py
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Testing
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_roslz4 test/roslz4_test.cpp)
  target_link_libraries(test_roslz4 roslz4 ${catkin_LIBRARIES})
endif()
