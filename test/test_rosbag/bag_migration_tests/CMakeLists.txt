# This is going to need some work because the bag files on
# download.ros.org all have messages with the test_rosbag package
# name.  Of course the package name for this package now is rosbag.
# If we can have a way to pass an override package name into
# generate_messages() it might make it work, or we can migrate the bag
# files themselves.

add_message_files(
  DIRECTORY msg_current
  FILES
  Constants.msg
  Converged.msg
  MigratedAddSub.msg
  MigratedExplicit.msg
  MigratedImplicit.msg
  MigratedMixed.msg
  PartiallyMigrated.msg
  Renamed4.msg
  Simple.msg
  SimpleMigrated.msg
  SubUnmigrated.msg
  Unmigrated.msg
)
generate_messages(DEPENDENCIES rosgraph_msgs std_msgs)

catkin_download_test_data(download_data_test_constants_gen1.bag http://download.ros.org/data/test_rosbag/constants_gen1.bag FILENAME test/constants_gen1.bag MD5 77ec8cb20e823ee3f3a87d07ea1132df )
catkin_download_test_data(download_data_test_constants_gen2.bag http://download.ros.org/data/test_rosbag/constants_gen2.bag FILENAME test/constants_gen2.bag MD5 4efd8459bf9403d1171dff06abcd6548 )
catkin_download_test_data(download_data_test_converged_gen1.bag http://download.ros.org/data/test_rosbag/converged_gen1.bag FILENAME test/converged_gen1.bag MD5 8e3524157d31b5761ac951fe16e03e12 )
catkin_download_test_data(download_data_test_converged_gen2.bag http://download.ros.org/data/test_rosbag/converged_gen2.bag FILENAME test/converged_gen2.bag MD5 0ad4041d2e3bab8262c12020ec3e048e )
catkin_download_test_data(download_data_test_converged_gen3.bag http://download.ros.org/data/test_rosbag/converged_gen3.bag FILENAME test/converged_gen3.bag MD5 90dd16cea5c51fca65be617654fb6b76 )
catkin_download_test_data(download_data_test_convergent_gen1.bag http://download.ros.org/data/test_rosbag/convergent_gen1.bag FILENAME test/convergent_gen1.bag MD5 a840aec95e50f3c828841b688dfab5a2 )
catkin_download_test_data(download_data_test_convergent_gen2.bag http://download.ros.org/data/test_rosbag/convergent_gen2.bag FILENAME test/convergent_gen2.bag MD5 fd84c135ba1548985b870d0d5f4113e9 )
catkin_download_test_data(download_data_test_migrated_addsub_gen1.bag http://download.ros.org/data/test_rosbag/migrated_addsub_gen1.bag FILENAME test/migrated_addsub_gen1.bag MD5 462e98cef72b5df56ce3feea9ddfc4d2 )
catkin_download_test_data(download_data_test_migrated_explicit_gen1.bag http://download.ros.org/data/test_rosbag/migrated_explicit_gen1.bag FILENAME test/migrated_explicit_gen1.bag MD5 0b77e45aec3ddb19a9f9771c945e2615 )
catkin_download_test_data(download_data_test_migrated_explicit_gen2.bag http://download.ros.org/data/test_rosbag/migrated_explicit_gen2.bag FILENAME test/migrated_explicit_gen2.bag MD5 b237680d6aae1b20355af097cf0f072b )
catkin_download_test_data(download_data_test_migrated_explicit_gen3.bag http://download.ros.org/data/test_rosbag/migrated_explicit_gen3.bag FILENAME test/migrated_explicit_gen3.bag MD5 8b883286e23779bbfc30b0e5588a4d64 )
catkin_download_test_data(download_data_test_migrated_implicit_gen1.bag http://download.ros.org/data/test_rosbag/migrated_implicit_gen1.bag FILENAME test/migrated_implicit_gen1.bag MD5 229d6cbd7dcec5e87fb2cf597b837243 )
catkin_download_test_data(download_data_test_migrated_implicit_gen2.bag http://download.ros.org/data/test_rosbag/migrated_implicit_gen2.bag FILENAME test/migrated_implicit_gen2.bag MD5 df60111e5f8034ba6259c4d76d07339b )
catkin_download_test_data(download_data_test_migrated_implicit_gen3.bag http://download.ros.org/data/test_rosbag/migrated_implicit_gen3.bag FILENAME test/migrated_implicit_gen3.bag MD5 d86f131070822f1673f7644bc28bfccd )
catkin_download_test_data(download_data_test_migrated_mixed_gen1.bag http://download.ros.org/data/test_rosbag/migrated_mixed_gen1.bag FILENAME test/migrated_mixed_gen1.bag MD5 2d0be37de2df6940b7bd93561dde7b51 )
catkin_download_test_data(download_data_test_migrated_mixed_gen2.bag http://download.ros.org/data/test_rosbag/migrated_mixed_gen2.bag FILENAME test/migrated_mixed_gen2.bag MD5 e000b2de3e4c417b0c9fbba9d2e92dc4 )
catkin_download_test_data(download_data_test_migrated_mixed_gen3.bag http://download.ros.org/data/test_rosbag/migrated_mixed_gen3.bag FILENAME test/migrated_mixed_gen3.bag MD5 6e1cbb529b7a45386589d00bf3908bdf )
catkin_download_test_data(download_data_test_partially_migrated_gen1.bag http://download.ros.org/data/test_rosbag/partially_migrated_gen1.bag FILENAME test/partially_migrated_gen1.bag MD5 dcfcf1e1c1f7ddc5ab723e171273a385 )
catkin_download_test_data(download_data_test_partially_migrated_gen2.bag http://download.ros.org/data/test_rosbag/partially_migrated_gen2.bag FILENAME test/partially_migrated_gen2.bag MD5 7b2d752559a7e3c724d8b2a032daf3c1 )
catkin_download_test_data(download_data_test_partially_migrated_gen3.bag http://download.ros.org/data/test_rosbag/partially_migrated_gen3.bag FILENAME test/partially_migrated_gen3.bag MD5 01161f84591bf956df7dff0a5e3fd2db )
catkin_download_test_data(download_data_test_renamed_gen1.bag http://download.ros.org/data/test_rosbag/renamed_gen1.bag FILENAME test/renamed_gen1.bag MD5 243d6109f3bfd2ad1af1337a776970e5 )
catkin_download_test_data(download_data_test_renamed_gen2.bag http://download.ros.org/data/test_rosbag/renamed_gen2.bag FILENAME test/renamed_gen2.bag MD5 229c79262e42b71f36e316ab1d1ad943 )
catkin_download_test_data(download_data_test_renamed_gen3.bag http://download.ros.org/data/test_rosbag/renamed_gen3.bag FILENAME test/renamed_gen3.bag MD5 c9c3d03807de367b533f5e0384b4d134 )
catkin_download_test_data(download_data_test_subunmigrated_gen1.bag http://download.ros.org/data/test_rosbag/subunmigrated_gen1.bag FILENAME test/subunmigrated_gen1.bag MD5 c78b85016aa34669e36d1eb5a7b5f2c7 )
catkin_download_test_data(download_data_test_unmigrated_gen1.bag http://download.ros.org/data/test_rosbag/unmigrated_gen1.bag FILENAME test/unmigrated_gen1.bag MD5 87d8d3a1c9679609533803709d21cea2 )

configure_file(test/migrate_test.py.in
  ${PROJECT_BINARY_DIR}/test/migrate_test.py)
catkin_add_nosetests(${PROJECT_BINARY_DIR}/test/migrate_test.py)
configure_file(test/random_record.xml.in
  ${PROJECT_BINARY_DIR}/test/random_record.xml)
add_rostest(${PROJECT_BINARY_DIR}/test/random_record.xml)
# Make sure that the random_record test runs before either of the random_play
# tests, both of which require a .bag file that the random_record test will
# write into /tmp. We're making an assumption about the target names generated
# by add_rostest(), but that naming scheme seems to be pretty stable.
configure_file(test/random_play.xml.in
  ${PROJECT_BINARY_DIR}/test/random_play.xml)
add_rostest(${PROJECT_BINARY_DIR}/test/random_play.xml
  DEPENDENCIES run_tests_test_rosbag_rostest_test_random_record.xml)
configure_file(test/random_play_sim.xml.in
  ${PROJECT_BINARY_DIR}/test/random_play_sim.xml)
add_rostest(${PROJECT_BINARY_DIR}/test/random_play_sim.xml
  DEPENDENCIES run_tests_test_rosbag_rostest_test_random_record.xml)
