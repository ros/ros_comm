cmake_minimum_required(VERSION 2.8.3)

project(test_roscpp)

find_package(catkin REQUIRED COMPONENTS
  message_generation roscpp rostest rosunit std_srvs
)

if(CATKIN_ENABLE_TESTING)
  find_package(Boost REQUIRED COMPONENTS signals filesystem system)

  include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

  # Testing message and service files.  See comment below about subdirectory support.
  add_message_files(
    NOINSTALL
    DIRECTORY test/msg
    FILES
    TestArray.msg
    TestEmpty.msg
    TestStringInt.msg
    TestWithHeader.msg
  )
  add_service_files(
    NOINSTALL
    DIRECTORY test/srv
    FILES
    BadTestStringString.srv
    TestStringString.srv
  )

  # Serialization testing message files.
  add_message_files(
    NOINSTALL
    DIRECTORY test_serialization/msg
    FILES
    ArrayOfFixedLength.msg
    ArrayOfVariableLength.msg
    Constants.msg
    CustomHeader.msg
    EmbeddedExternal.msg
    EmbeddedFixedLength.msg
    EmbeddedVariableLength.msg
    FixedLengthArrayOfExternal.msg
    FixedLength.msg
    FixedLengthStringArray.msg
    HeaderNotFirstMember.msg
    VariableLengthArrayOfExternal.msg
    VariableLength.msg
    VariableLengthStringArray.msg
    WithDuration.msg
    WithHeader.msg
    WithMemberNamedHeaderThatIsNotAHeader.msg
    WithTime.msg
  )

  # Performance testing message files.  add_message_files() does not
  # currently support being called from a subdirectory, so this is here
  # for now.
  add_message_files(
    NOINSTALL
    DIRECTORY perf/msg
    FILES
    LatencyMessage.msg
    ThroughputMessage.msg
  )

  # Message files for testing the performance of serialization.
  add_message_files(
    NOINSTALL
    DIRECTORY perf_serialization/msg
    FILES
    ChannelFloat32.msg
    Point32.msg
    PointCloud.msg
  )

  # dependencies std_msgs and rosgraph_msgs are needed by tests, but not by roscpp otherwise.
  generate_messages(DEPENDENCIES rosgraph_msgs std_msgs)
endif()

catkin_package(
  CATKIN_DEPENDS message_runtime rosconsole rosgraph_msgs std_msgs xmlrpcpp
  DEPENDS Boost
)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
  add_subdirectory(perf)
  add_subdirectory(test_serialization)
  add_subdirectory(perf_serialization)
endif()
