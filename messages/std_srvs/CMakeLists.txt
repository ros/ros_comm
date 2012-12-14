cmake_minimum_required(VERSION 2.8.3)
project(std_srvs)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_service_files(DIRECTORY srv
  FILES Empty.srv)

generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)
