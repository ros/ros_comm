cmake_minimum_required(VERSION 2.8.3)
project(rospy)
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

# install legacy infrastructure needed by rosbuild
install(FILES rosbuild/rospy.cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rosbuild)
catkin_install_python(PROGRAMS
  rosbuild/scripts/genmsg_py.py
  rosbuild/scripts/gensrv_py.py
  rosbuild/scripts/genutil.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rosbuild/scripts)
catkin_install_python(PROGRAMS
  test_nodes/talker.py
  test_nodes/listener.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test_nodes/)
