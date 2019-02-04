cmake_minimum_required(VERSION 2.8.3)

project(test_rosmaster)

find_package(catkin REQUIRED COMPONENTS genmsg rostest std_msgs)

if(CATKIN_ENABLE_TESTING)
  add_message_files(DIRECTORY msg
    FILES
    Arrays.msg
    Composite.msg
    CompositeA.msg
    CompositeB.msg
    Embed.msg
    Empty.msg
    Floats.msg
    RosmsgA.msg
    RosmsgB.msg
    RosmsgC.msg
    Simple.msg
    String.msg
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
endif()

catkin_package()

if(CATKIN_ENABLE_TESTING)
  add_rostest(test/rosmaster.test)
  add_rostest(test/paramserver.test)
endif()
