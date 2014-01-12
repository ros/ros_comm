# ros_comm/tools/rosconsole/cmake/rosconsole-extras.cmake

# force automatic escaping of preprocessor definitions
cmake_policy(PUSH)
cmake_policy(SET CMP0005 NEW)

# add ROS_PACKAGE_NAME define required by the named logging macros
add_definitions(-DROS_PACKAGE_NAME=\"${PROJECT_NAME}\")

if("@ROSCONSOLE_BACKEND@" STREQUAL "log4cxx")
  # add ROSCONSOLE_BACKEND_LOG4CXX define required for backward compatible log4cxx symbols
  add_definitions(-DROSCONSOLE_BACKEND_LOG4CXX)
endif()

cmake_policy(POP)
