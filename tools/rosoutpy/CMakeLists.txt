cmake_minimum_required(VERSION 2.8.3)
project(rosoutpy)

find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS rospy rosgraph_msgs)

catkin_install_python(PROGRAMS
  scripts/rosoutpy
  scripts/rosoutpy.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
install(FILES
  launch/rosoutpy.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(FILES
  README
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)
