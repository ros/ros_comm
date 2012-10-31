cmake_minimum_required(VERSION 2.8.3)
project(test_rosmaster)

find_package(catkin REQUIRED COMPONENTS genmsg rostest)

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

generate_messages()

catkin_package()

add_rostest(test/rosmaster.test)
add_rostest(test/paramserver.test)
