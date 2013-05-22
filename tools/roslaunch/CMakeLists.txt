cmake_minimum_required(VERSION 2.8.3)
project(roslaunch)
find_package(catkin REQUIRED)
catkin_package()

if(CMAKE_HOST_UNIX)
  catkin_add_env_hooks(10.roslaunch SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
  catkin_add_env_hooks(20.roslaunch SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
else()
  catkin_add_env_hooks(10.roslaunch SHELLS bat DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
endif()

catkin_python_setup()

install(FILES resources/roscore.xml
  DESTINATION ${CATKIN_GLOBAL_ETC_DESTINATION}/ros)

# install example launch files
install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_add_nosetests(test/unit)
