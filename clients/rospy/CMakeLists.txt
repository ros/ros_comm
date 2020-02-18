cmake_minimum_required(VERSION 3.0.2)
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
