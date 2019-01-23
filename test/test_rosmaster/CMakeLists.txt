cmake_minimum_required(VERSION 2.8.3)

project(test_rosmaster)

find_package(catkin REQUIRED COMPONENTS genmsg rostest std_msgs)

if(CATKIN_ENABLE_TESTING)
  add_message_files(DIRECTORY msg)
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
