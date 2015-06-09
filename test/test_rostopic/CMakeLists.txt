cmake_minimum_required(VERSION 2.8.3)

project(test_rostopic)

find_package(catkin REQUIRED COMPONENTS genmsg rostest std_msgs)

if(CATKIN_ENABLE_TESTING)
  add_message_files(DIRECTORY msg
    FILES
    Arrays.msg
    ArrayVal.msg
    Embed.msg
    Floats.msg
    Simple.msg
    TVals.msg
    Val.msg
  )
  generate_messages(DEPENDENCIES std_msgs)
endif()

catkin_package()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
