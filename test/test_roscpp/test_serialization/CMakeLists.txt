if(NOT GTEST_FOUND)
  return()
endif()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

catkin_add_gtest(${PROJECT_NAME}-serialization src/serialization.cpp)
if(TARGET ${PROJECT_NAME}-serialization)
  target_link_libraries(${PROJECT_NAME}-serialization ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
  add_dependencies(${PROJECT_NAME}-serialization ${${PROJECT_NAME}_EXPORTED_TARGETS})
endif()

catkin_add_gtest(${PROJECT_NAME}-generated_messages src/generated_messages.cpp)
if(TARGET ${PROJECT_NAME}-generated_messages)
  target_link_libraries(${PROJECT_NAME}-generated_messages ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
  add_dependencies(${PROJECT_NAME}-generated_messages ${${PROJECT_NAME}_EXPORTED_TARGETS})
endif()

add_executable(${PROJECT_NAME}-builtin_types EXCLUDE_FROM_ALL src/builtin_types.cpp)
add_dependencies(tests ${PROJECT_NAME}-builtin_types) 
target_link_libraries(${PROJECT_NAME}-builtin_types ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
add_dependencies(${PROJECT_NAME}-builtin_types ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_rostest(test/builtin_types.test)

add_executable(${PROJECT_NAME}-pre_deserialize EXCLUDE_FROM_ALL src/pre_deserialize.cpp)
add_dependencies(tests ${PROJECT_NAME}-pre_deserialize)
target_link_libraries(${PROJECT_NAME}-pre_deserialize ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
add_dependencies(${PROJECT_NAME}-pre_deserialize ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_rostest(test/pre_deserialize.test)
