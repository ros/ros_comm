find_package(catkin REQUIRED)

_generate_function_if_testing_is_disabled("add_rostest")

include(CMakeParseArguments)

function(add_rostest file)
  _warn_if_skip_testing("add_rostest")

@[if DEVELSPACE]@
  # bin in develspace
  set(ROSTEST_EXE "@(PROJECT_SOURCE_DIR)/scripts/rostest")
@[else]@
  # bin in installspace
  set(ROSTEST_EXE "${rostest_DIR}/../../../@(CATKIN_GLOBAL_BIN_DESTINATION)/rostest")
@[end if]@

  cmake_parse_arguments(_rostest "" "WORKING_DIRECTORY" "ARGS;DEPENDENCIES" ${ARGN})

  # Check that the file exists, #1621
  set(_file_name _file_name-NOTFOUND)
  if(IS_ABSOLUTE ${file})
    set(_file_name ${file})
  else()
    find_file(_file_name ${file}
              PATHS ${CMAKE_CURRENT_SOURCE_DIR}
              NO_DEFAULT_PATH
              NO_CMAKE_FIND_ROOT_PATH)  # for cross-compilation.  thanks jeremy.
    if(NOT _file_name)
      message(FATAL_ERROR "Can't find rostest file \"${file}\"")
    endif()
  endif()

  # strip PROJECT_SOURCE_DIR and PROJECT_BINARY_DIR from absolute filename to get unique test name (as rostest does it internally)
  set(_testname ${_file_name})
  rostest__strip_prefix(_testname "${PROJECT_SOURCE_DIR}/")
  rostest__strip_prefix(_testname "${PROJECT_BINARY_DIR}/")

  # to support registering the same test with different ARGS
  # append the args to the test name
  if(_rostest_ARGS)
    get_filename_component(_ext ${_testname} EXT)
    get_filename_component(_testname ${_testname} NAME_WE)
    foreach(arg ${_rostest_ARGS})
      string(REPLACE ":=" "_" arg_string "${arg}")
      set(_testname "${_testname}__${arg_string}")
    endforeach()
    set(_testname "${_testname}${_ext}")
  endif()

  string(REPLACE "/" "_" _testname ${_testname})

  get_filename_component(_output_name ${_testname} NAME_WE)
  set(_output_name "${_output_name}.xml")
  string(REPLACE ";" " " _rostest_ARGS "${_rostest_ARGS}")
  set(cmd "${ROSTEST_EXE} --pkgdir=${PROJECT_SOURCE_DIR} --package=${PROJECT_NAME} --results-filename ${_output_name} --results-base-dir \"${CATKIN_TEST_RESULTS_DIR}\" ${_file_name} ${_rostest_ARGS}")
  catkin_run_tests_target("rostest" ${_testname} "rostest-${_output_name}" COMMAND ${cmd} WORKING_DIRECTORY ${_rostest_WORKING_DIRECTORY} DEPENDENCIES ${_rostest_DEPENDENCIES})
endfunction()

# This is an internal function, use add_rostest_gtest or
# add_rostest_gmock instead
#
# The function arguments are the same as for add_rostest_gtest
# and add_rostest_gmock
#
function(_add_rostest_google_test target)
  set(multiValueArgs LAUNCH_FILES SOURCES LIBRARIES DEPENDENCIES)
  cmake_parse_arguments(_rostest_google_test "DEPRECATED" "TYPE" "${multiValueArgs}" ${ARGN})

  if(NOT "${_rostest_google_test_TYPE}" STREQUAL "gtest" AND NOT "${_rostest_google_test_TYPE}" STREQUAL "gmock")
    message(FATAL_ERROR
      "Invalid use of _rostest_google_test function, "
      "TYPE argument must be 'gtest' or 'gmock'")
    return()
  endif()

  if(_rostest_google_test_DEPRECATED)
    message(STATUS "Syntax add_rostest_${_rostest_google_test_TYPE}(<target> <launch_file> <source>...) is deprecated.")
    message(STATUS "Use syntax add_rostest_${_rostest_google_test_TYPE}(<target> LAUNCH_FILES <launch_file>... SOURCES <source>...) instead.")
    set(_rostest_google_test_SOURCES ${_rostest_google_test_UNPARSED_ARGUMENTS})
    list(GET         _rostest_google_test_SOURCES 0 _rostest_google_test_LAUNCH_FILES)
    list(REMOVE_AT   _rostest_google_test_SOURCES 0)
  endif()

  string(TOUPPER "${_rostest_google_test_TYPE}" type_upper)
  if("${_rostest_google_test_LAUNCH_FILES}" STREQUAL "")
    message(FATAL_ERROR "add_rostest_${_rostest_google_test_TYPE}() needs at least one launch file to execute test ${type_upper}")
  endif()
  if("${_rostest_google_test_SOURCES}" STREQUAL "")
    message(FATAL_ERROR "add_rostest_${_rostest_google_test_TYPE}() needs at least one file argument to compile a ${type_upper} executable")
  endif()

  if(${type_upper}_FOUND)
    include_directories(${${type_upper}_INCLUDE_DIRS})
    add_executable(${target} EXCLUDE_FROM_ALL ${_rostest_google_test_SOURCES})
    target_link_libraries(${target} ${${type_upper}_LIBRARIES} ${_rostest_google_test_LIBRARIES})
    if(_rostest_google_test_DEPENDENCIES)
      add_dependencies(${target} ${_rostest_google_test_DEPENDENCIES})
    endif()
    if(TARGET tests)
      add_dependencies(tests ${target})
    endif()
    foreach(launch_file ${_rostest_google_test_LAUNCH_FILES})
      add_rostest(${launch_file} DEPENDENCIES ${target})
    endforeach()
  else()
    message(STATUS "Skipped add_rostest_${_rostest_google_test_TYPE}(${target}) due to '${_rostest_google_test_TYPE}' is not found")
  endif()
endfunction()

#
# Register the launch file with add_rostest() and compile all
# passed files into a GTest binary.
#
# .. note:: The function does nothing if GTest was not found.  The
#   target is only compiled when tests are built and linked against
#   the GTest libraries.
#
# ::
#   add_rostest_gtest(<target>
#     LAUNCH_FILES <launch_file1> [<launch_file2> ...]
#     SOURCES <source1> [<source2> ...]
#     [LIBRARIES <library1> [<library2> ...]]
#     [DEPENDENCIES <dependency1> [<dependency2> ...]])
#
# :param target: target name of the GTest executable
# :type target: string
#
# The function arguments are:
#
# ``LAUNCH_FILES``
#   a list of relative paths to the roslaunch files which will be applied in `add_rostest` for <target>
# ``SOURCES``
#   a list of source files to compile into GTest executable <target>
# ``LIBRARIES``
#   a list of libraries to link with GTest executable <target>
# ``DEPENDENCIES``
#   a list of dependencies of GTest executable <target>
#
function(add_rostest_gtest target)
  set(_type "gtest")
  set(multiValueArgs LAUNCH_FILES SOURCES)
  cmake_parse_arguments(_rostest_${_type} "" "" "${multiValueArgs}" ${ARGN})
  if (NOT _rostest${_type}_LAUNCH_FILES AND NOT _rostest${_type}_SOURCES)
    _add_rostest_google_test(${target} TYPE ${_type} DEPRECATED ${ARGN})
  else()
    _add_rostest_google_test(${target} TYPE ${_type} ${ARGN})
  endif()
endfunction()

#
# Register the launch file with add_rostest() and compile all
# passed files into a GMock binary.
#
# .. note:: The function does nothing if GMock was not found.  The
#   target is only compiled when tests are built and linked against
#   the GMock libraries.
#
# ::
#   add_rostest_gmock(<target>
#     LAUNCH_FILES <launch_file1> [<launch_file2> ...]
#     SOURCES <source1> [<source2> ...]
#     [LIBRARIES <library1> [<library2> ...]]
#     [DEPENDENCIES <dependency1> [<dependency2> ...]])
#
# :param target: target name of the GTest executable
# :type target: string
#
# The function arguments are:
#
# ``LAUNCH_FILES``
#   a list of relative paths to the roslaunch files which will be applied in `add_rostest` with <target>
# ``SOURCES``
#   a list of source files to compile into GMock executable <target>
# ``LIBRARIES``
#   a list of libraries to link with GMock executable <target>
# ``DEPENDENCIES``
#   a list of dependencies of GMock executable <target>
#
function(add_rostest_gmock target)
  set(_type "gmock")
  set(multiValueArgs LAUNCH_FILES SOURCES)
  cmake_parse_arguments(_rostest_${_type} "" "" "${multiValueArgs}" ${ARGN})
  if (NOT _rostest${_type}_LAUNCH_FILES AND NOT _rostest${_type}_SOURCES)
    _add_rostest_google_test(${target} TYPE ${_type} DEPRECATED ${ARGN})
  else()
    _add_rostest_google_test(${target} TYPE ${_type} ${ARGN})
  endif()
endfunction()

macro(rostest__strip_prefix var prefix)
  string(LENGTH ${prefix} prefix_length)
  string(LENGTH ${${var}} var_length)
  if(${var_length} GREATER ${prefix_length})
    string(SUBSTRING "${${var}}" 0 ${prefix_length} var_prefix)
    if("${var_prefix}" STREQUAL "${prefix}")
      # passing length -1 does not work for CMake < 2.8.5
      # http://public.kitware.com/Bug/view.php?id=10740
      string(LENGTH "${${var}}" _rest)
      math(EXPR _rest "${_rest} - ${prefix_length}")
      string(SUBSTRING "${${var}}" ${prefix_length} ${_rest} ${var})
    endif()
  endif()
endmacro()
