cmake_minimum_required(VERSION 2.8.3)
project(rosgraph)
find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

# logging config file goes in both etc and package layout (for now).
# want to get rid of package layout copy, but need to be able to
# locate etc first.
install(FILES conf/python_logging.conf
  DESTINATION ${CATKIN_GLOBAL_ETC_DESTINATION}/ros)
install(FILES conf/python_logging.conf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/conf)
install(PROGRAMS scripts/rosgraph
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
