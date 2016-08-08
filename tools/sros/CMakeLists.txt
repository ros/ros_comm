cmake_minimum_required(VERSION 2.8.3)
project(sros)
find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

# logging config file goes in both etc and package layout (for now).
# want to get rid of package layout copy, but need to be able to
# locate etc first.
install(FILES conf/keyserver_config.yaml
  DESTINATION ${CATKIN_GLOBAL_ETC_DESTINATION}/ros)
install(FILES conf/keyserver_config.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/conf)

install(FILES conf/policy_config.yaml
  DESTINATION ${CATKIN_GLOBAL_ETC_DESTINATION}/ros)
install(FILES conf/policy_config.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/conf)

catkin_install_python(PROGRAMS scripts/sros
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/sroscore
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/sroskeyserver
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/sroslaunch
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#if(CATKIN_ENABLE_TESTING)
#  catkin_add_nosetests(test)
#endif()
