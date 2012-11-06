find_package(catkin REQUIRED)

include(CMakeParseArguments)

function(add_rostest file)
@[if DEVELSPACE]@
  # find program in develspace
  find_program_required(ROSTEST_EXE rostest 
    PATHS @(PROJECT_SOURCE_DIR)/scripts
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
@[else]@
  # find program in installspace
  find_program_required(ROSTEST_EXE rostest 
    PATHS @(CMAKE_INSTALL_PREFIX)/bin
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
@[end if]@

  cmake_parse_arguments(_rostest "" "WORKING_DIRECTORY" "" ${ARGN})

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

  string(REPLACE "/" "_" _testname ${_testname})
  get_filename_component(_output_name ${_testname} NAME_WE)
  set(_output_name "${_output_name}.xml")
  set(cmd "${ROSTEST_EXE} --pkgdir=${PROJECT_SOURCE_DIR} --package=${PROJECT_NAME} ${_file_name}")
  catkin_run_tests_target("rostest" ${_testname} "rostest-${_output_name}" COMMAND ${cmd} WORKING_DIRECTORY ${_rostest_WORKING_DIRECTORY})
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
