#!/bin/bash

# Software License Agreement (BSD License)
#
# Copyright (c) 2016, TORK All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TORK (Tokyo Opensource Robotics Kyokai Association) 
#    nor the names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

FILENAME_LOG_COMMANDS=/tmp/ros_diagnosisinfo_commands_`date +"%Y%m%d-%H%M%S"`.log
FILENAME_LOG_ROS=/tmp/ros_diagnosisinfo_ros_`date +"%Y%m%d-%H%M%S"`.tgz
FILENAME_LOG_ALL=/tmp/ros_diagnosisinfo_all_`date +"%Y%m%d-%H%M%S"`.tgz

env | grep -E '(ROS|PATH)' | sort | tee -a ${FILENAME_LOG_COMMANDS}
ifconfig | tee -a ${FILENAME_LOG_COMMANDS}
## Get packages' version. Ref. http://askubuntu.com/a/347563/24203
## Get rosversion of the same packages
res_dpkg=$(dpkg -l | grep '^ii' | grep ros- | awk '{print $2 " " $3}')
IFS=$'\n'  # http://askubuntu.com/questions/344407/how-to-read-complete-line-in-for-loop-with-spaces
for i in ${res_dpkg}
do 
  printf "%s $i " 2>&1 | tee -a ${FILENAME_LOG_COMMANDS}
  p_underscore=$(echo ${i} | awk '{print $1}' | sed 's/ros-\([a-zA-Z]*\)-//' | tr '-' '_');
  printf "%s ${p_underscore} " 2>&1 | tee -a ${FILENAME_LOG_COMMANDS}
  rospackfind_result=$(rospack -q find ${p_underscore})
  printf "%s ${rospackfind_result} " 2>&1 | tee -a ${FILENAME_LOG_COMMANDS}
  echo "${rospackfind_result}" | grep -o '[^/]*$' | xargs rosversion 2>&1 | tee -a ${FILENAME_LOG_COMMANDS}
  if [[ -z "$rospackfind_result" ]]; then
    continue;
  fi
done

res_dpkg=$(rospack list)
IFS=$'\n'  # http://askubuntu.com/questions/344407/how-to-read-complete-line-in-for-loop-with-spaces
for i in ${res_dpkg}
do
  p_packagename=$(echo ${i} | awk '{print $1}');
  printf "* %s ${p_packagename} " 2>&1 | tee -a ${FILENAME_LOG_COMMANDS}
  rospackfind_result=$(rospack -q find ${p_packagename})
  printf "%s ${rospackfind_result} " 2>&1 | tee -a ${FILENAME_LOG_COMMANDS}
  echo "${rospackfind_result}" | grep -o '[^/]*$' | xargs rosversion 2>&1 | tee -a ${FILENAME_LOG_COMMANDS}
  if [[ -z "$rospackfind_result" ]]; then
    continue;
  fi
  # git status
  cd "${rospackfind_result}";
  if [ `git diff 2> /dev/null | wc -l` != "0" ]; then
      git status | tee -a ${FILENAME_LOG_COMMANDS}
  fi
  cd $OLDPWD
done

tar -C ~/.ros/log -cvzf ${FILENAME_LOG_ROS} `cd ~/.ros/log; ls -d * | head -1`

tar cfz ${FILENAME_LOG_ALL} ${FILENAME_LOG_COMMANDS} ${FILENAME_LOG_ROS}

echo "=== All diagnostic info recorded into a tarball: ${FILENAME_LOG_ALL}"
echo "=== If you have any questions regarding this script, ask at https://github.com/ros/ros_comm/issues"
