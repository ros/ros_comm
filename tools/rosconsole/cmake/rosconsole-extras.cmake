# ros_comm/tools/rosconsole/cmake/rosconsole-extras.cmake

# force automatic escaping of preprocessor definitions
cmake_policy(PUSH)
cmake_policy(SET CMP0005 NEW)

# add ROS_PACKAGE_NAME define required by the named logging macros
add_definitions(-DROS_PACKAGE_NAME=\"${PROJECT_NAME}\")

cmake_policy(POP)
