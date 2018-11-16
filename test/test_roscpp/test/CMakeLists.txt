catkin_add_gtest(${PROJECT_NAME}-test_version test_version.cpp)
if(TARGET ${PROJECT_NAME}-test_version)
  target_link_libraries(${PROJECT_NAME}-test_version ${catkin_LIBRARIES})
endif()

catkin_add_gtest(${PROJECT_NAME}-test_header test_header.cpp)
if(TARGET ${PROJECT_NAME}-test_header)
  target_link_libraries(${PROJECT_NAME}-test_header ${catkin_LIBRARIES})
endif()

catkin_add_gtest(${PROJECT_NAME}-test_poll_set test_poll_set.cpp)
if(TARGET ${PROJECT_NAME}-test_poll_set)
  target_link_libraries(${PROJECT_NAME}-test_poll_set ${catkin_LIBRARIES})
  if(WIN32)
    target_link_libraries(${PROJECT_NAME}-test_poll_set ws2_32)
  endif()
endif()

catkin_add_gtest(${PROJECT_NAME}-test_transport_tcp test_transport_tcp.cpp)
if(TARGET ${PROJECT_NAME}-test_transport_tcp)
  target_link_libraries(${PROJECT_NAME}-test_transport_tcp ${catkin_LIBRARIES})
endif()

catkin_add_gtest(${PROJECT_NAME}-test_subscription_queue test_subscription_queue.cpp)
if(TARGET ${PROJECT_NAME}-test_subscription_queue)
  target_link_libraries(${PROJECT_NAME}-test_subscription_queue ${catkin_LIBRARIES})
endif()

catkin_add_gtest(${PROJECT_NAME}-test_callback_queue test_callback_queue.cpp)
if(TARGET ${PROJECT_NAME}-test_callback_queue)
  target_link_libraries(${PROJECT_NAME}-test_callback_queue ${catkin_LIBRARIES})
endif()

catkin_add_gtest(${PROJECT_NAME}-test_names test_names.cpp)
if(TARGET ${PROJECT_NAME}-test_names)
  target_link_libraries(${PROJECT_NAME}-test_names ${catkin_LIBRARIES})
endif()

catkin_add_gtest(${PROJECT_NAME}-test_args test_args.cpp)
if(TARGET ${PROJECT_NAME}-test_args)
  target_link_libraries(${PROJECT_NAME}-test_args ${catkin_LIBRARIES})
endif()

if(GTEST_FOUND)
  add_subdirectory(src)
endif()

add_rostest(launch/real_time_test.xml)
add_rostest(launch/sim_time_test.xml)

# Publish one message
add_rostest(launch/pubsub_once.xml)

# Publish a bunch of messages back to back
add_rostest(launch/pubsub_n_fast.xml)
add_rostest(launch/pubsub_n_fast_udp.xml)

# Publish a bunch of empty messages
add_rostest(launch/pubsub_empty.xml)

# Publish only to the subscriber from the subscriber callback
add_rostest(launch/pub_onsub.xml)

# Publish a bunch of large messages back to back
add_rostest(launch/pubsub_n_fast_large_message.xml)

# Subscribe, listen, unsubscribe, re-subscribe to a different topic, listen
# again
add_rostest(launch/pubsub_resub_once.xml)

# Subscribe and unsubscribe repeatedly, ensuring that callbacks don't get
# called when not subscribed.
add_rostest(launch/pubsub_unsub.xml)

# Advertise, then unadvertise, and ensure that subscriber callback doesn't
# get invoked afterward, while a subscriber is constantly subscribing and
# unsubscribing
add_rostest(launch/pubsub_unadv.xml)

# Call a service
add_rostest(launch/service_call.xml)

# disabling the test again since it does not work realiable
#add_rostest(launch/service_deadlock.xml)

add_rostest(launch/service_exception.xml)

add_rostest(launch/service_call_unadv.xml)

add_rostest(launch/service_call_zombie.xml)

# Repeatedly call ros::init() and ros::fini()
add_rostest(launch/multiple_init_fini.xml)

# Test node inspection functionality
add_rostest(launch/inspection.xml)

# Test that advertising a service multiple times fails
add_rostest(launch/service_adv_multiple.xml)

# Test that the second node to advertise a service "wins"
add_rostest(launch/service_multiple_providers.xml)

# Test namespaces
add_rostest(launch/namespaces.xml)

# Test command-line name remapping
add_rostest(launch/name_remapping.xml)

add_rostest(launch/name_not_remappable.xml)

# Test command-line name remapping
add_rostest(launch/name_remapping_ROS_NAMESPACE.xml)

# Test params
add_rostest(launch/params.xml)

# Test getting information from the master
add_rostest(launch/get_master_information.xml)

# Test multiple subscriptions
add_rostest(launch/multiple_subscriptions.xml)

add_rostest(launch/pingpong.xml)
add_rostest(launch/pingpong_large.xml)

add_rostest(launch/subscribe_self.xml)

add_rostest(launch/check_master.xml)
#add_rostest(launch/check_master_false.xml)

add_rostest(launch/handles.xml)
add_rostest(launch/timer_callbacks.xml)
add_rostest(launch/latching_publisher.xml)
add_rostest(launch/loads_of_publishers.xml)
add_rostest(launch/incrementing_sequence.xml)
add_rostest(launch/subscription_callback_types.xml)
add_rostest(launch/service_callback_types.xml)
add_rostest(launch/intraprocess_subscriptions.xml)
add_rostest(launch/nonconst_subscriptions.xml)
add_rostest(launch/subscribe_retry_tcp.xml)
add_rostest(launch/subscribe_star.xml)
add_rostest(launch/parameter_validation.xml)

add_rostest(launch/no_remappings.xml)
add_rostest(launch/local_remappings.xml)
add_rostest(launch/global_remappings.xml)
add_rostest(launch/ns_node_remapping.xml)
add_rostest(launch/search_param.xml)

add_rostest(launch/stamped_topic_statistics_with_empty_timestamp.xml)
add_rostest(launch/topic_statistic_frequency.xml DEPENDENCIES ${PROJECT_NAME}-publisher_rate ${PROJECT_NAME}-subscriber ${PROJECT_NAME}-topic_statistic_frequency)

# Test multiple latched publishers within the same process
add_rostest(launch/multiple_latched_publishers.xml)

# Test spinners
add_rostest(launch/spinners.xml)
