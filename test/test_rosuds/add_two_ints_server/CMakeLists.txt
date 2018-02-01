include_directories(${catkin_INCLUDE_DIRS})
add_executable(add_two_ints_server add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
install(TARGETS add_two_ints_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
