catkin_add_gtest(xmlrpcvalue_base64 xmlrpcvalue_base64.cpp)
if(TARGET xmlrpcvalue_base64)
  target_link_libraries(xmlrpcvalue_base64 xmlrpcpp)
endif()

if(NOT WIN32)
catkin_add_gtest(test_socket
  test_socket.cpp
  test_system_mocks.c
  ../src/XmlRpcSocket.cpp
  ../src/XmlRpcUtil.cpp
)
set_target_properties(test_socket PROPERTIES
  LINK_FLAGS
  "-Wl,--wrap=accept -Wl,--wrap=bind -Wl,--wrap=close -Wl,--wrap=connect -Wl,--wrap=getaddrinfo -Wl,--wrap=getsockname -Wl,--wrap=listen -Wl,--wrap=read -Wl,--wrap=setsockopt -Wl,--wrap=select -Wl,--wrap=socket -Wl,--wrap=write -Wl,--wrap=fcntl -Wl,--wrap=freeaddrinfo"
)
endif()

catkin_add_gtest(TestValues TestValues.cpp)
target_link_libraries(TestValues xmlrpcpp)

catkin_add_gtest(test_util test_util.cpp)
target_link_libraries(test_util xmlrpcpp)

catkin_add_gtest(TestXml TestXml.cpp)
target_link_libraries(TestXml xmlrpcpp)
