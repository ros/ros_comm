# generated from ros_comm/tools/roslaunch/cmake/roslaunch-extras.cmake.em

@[if DEVELSPACE]@
# set path to roslaunch-check script in develspace
set(roslaunch_check_script @(CMAKE_CURRENT_SOURCE_DIR)/scripts/roslaunch-check)
@[else]@
# set path to roslaunch-check script installspace
set(roslaunch_check_script ${roslaunch_DIR}/../scripts/roslaunch-check)
@[end if]@

#
# Check ROS launch files for validity as part of the unit tests.
#
# :param path: the path to a launch file or a directory containing launch files
#   either relative to th current CMakeLists file or absolute
# :type url: string
#
# :param ARGV: arbitrary arguments in the form 'key=value'
#   which will be set as environment variables
# :type ARGV: string
#
function(roslaunch_add_file_check path)
  if(IS_ABSOLUTE ${path})
    set(abspath ${path})
  else()
    set(abspath "${CMAKE_CURRENT_SOURCE_DIR}/${path}")
  endif()
  if(NOT EXISTS ${abspath})
    message(FATAL_ERROR "roslaunch_add_file_check() path '${abspath}' was not found")
  endif()

  string(REPLACE "/" "_" testname ${path})
  set(output_path ${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME})
  set(cmd "${CMAKE_COMMAND} -E make_directory ${output_path}")
  set(output_file_name "roslaunch-check_${testname}.xml")
  set(cmd ${cmd} "${roslaunch_check_script} -o '${output_path}/${output_file_name}' '${abspath}' ${ARGN}")
  catkin_run_tests_target("roslaunch-check" ${testname} "${output_file_name}" COMMAND ${cmd})
endfunction()
