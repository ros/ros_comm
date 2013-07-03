cmake_minimum_required(VERSION 2.8.3)

if(NOT CATKIN_ENABLE_TESTING)
  return()
endif()

project(test_rosmaster)

find_package(catkin REQUIRED COMPONENTS genmsg rostest std_msgs)

add_message_files(DIRECTORY msg
  FILES
  String.msg
  Arrays.msg
  CompositeA.msg CompositeB.msg Composite.msg
  Embed.msg Floats.msg Simple.msg
  RosmsgA.msg
  RosmsgB.msg
  RosmsgC.msg
  TestArrays.msg
  TestHeader.msg
  TestPrimitives.msg
  TestString.msg
  TVals.msg
)
add_service_files(DIRECTORY srv
  FILES
  AddTwoInts.srv
  RossrvA.srv
  RossrvB.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

add_rostest(test/rosmaster.test)
add_rostest(test/paramserver.test)
