catkin_add_gtest(xmlrpcvalue_base64 xmlrpcvalue_base64.cpp)
if(TARGET xmlrpcvalue_base64)
  target_link_libraries(xmlrpcvalue_base64 xmlrpcpp)
endif()
