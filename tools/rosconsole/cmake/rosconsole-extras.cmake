# ros_comm/tools/rosconsole/cmake/rosconsole-extras.cmake
cmake_policy(VERSION 2.8.3)

# add ROS_PACKAGE_NAME define required by the named logging macros
add_definitions(-DROS_PACKAGE_NAME=\"${PROJECT_NAME}\")
