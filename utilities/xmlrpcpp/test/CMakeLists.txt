catkin_add_gtest(xmlrpcvalue_base64 xmlrpcvalue_base64.cpp)
if(TARGET xmlrpcvalue_base64)
  target_link_libraries(xmlrpcvalue_base64 xmlrpcpp)
endif()

catkin_add_gtest(TestValues TestValues.cpp)
target_link_libraries(TestValues xmlrpcpp)

catkin_add_gtest(test_util test_util.cpp)
target_link_libraries(test_util xmlrpcpp)

catkin_add_gtest(TestXml TestXml.cpp)
target_link_libraries(TestXml xmlrpcpp)
