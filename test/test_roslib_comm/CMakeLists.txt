cmake_minimum_required(VERSION 2.8.3)
project(test_roslib_comm)
find_package(catkin REQUIRED COMPONENTS genmsg rosgraph_msgs std_msgs)

add_message_files(DIRECTORY msg
  FILES
  ArrayOfMsgs.msg
  FieldNameChange1.msg
  FieldNameChange2.msg
  FillEmbedTime.msg
  FillSimple.msg
  HeaderTest.msg
  SameSubMsg1.msg
  SameSubMsg2.msg
  SameSubMsg3.msg
  TypeNameChange1.msg
  TypeNameChange2.msg
  TypeNameChangeArray1.msg
  TypeNameChangeArray2.msg
  TypeNameChangeComplex1.msg
  TypeNameChangeComplex2.msg
  NOINSTALL
  )
generate_messages(DEPENDENCIES rosgraph_msgs std_msgs)

catkin_package()

catkin_add_nosetests(test)
