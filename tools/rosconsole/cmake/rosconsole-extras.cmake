# ros_comm/tools/rosconsole/cmake/rosconsole-extras.cmake

# add ROS_PACKAGE_NAME define required by the named logging macros
add_definitions(-DROS_PACKAGE_NAME="\\"${PROJECT_NAME}\\"")
