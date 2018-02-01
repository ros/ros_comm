include_directories(${catkin_INCLUDE_DIRS})
add_executable(add_two_ints_client add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
install(TARGETS add_two_ints_client
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
