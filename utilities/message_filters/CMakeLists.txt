cmake_minimum_required(VERSION 3.5)
project(message_filters)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED)

include_directories(include 
  ${rclcpp_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/connection.cpp)
target_link_libraries(${PROJECT_NAME} 
  ${rclcpp_LIBRARIES}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h")

