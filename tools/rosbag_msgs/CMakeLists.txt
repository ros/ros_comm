cmake_minimum_required(VERSION 2.8.3)
project(rosbag_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation rosgraph_msgs)

add_message_files(DIRECTORY msg FILES SnapshotStatus.msg)
add_service_files(DIRECTORY srv FILES TriggerSnapshot.srv)

generate_messages(DEPENDENCIES rosgraph_msgs)

catkin_package(CATKIN_DEPENDS message_runtime rosgraph_msgs)
