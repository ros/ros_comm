#!/bin/bash

SCRIPT_DIR=$(dirname $(readlink -f $0))

# check roscore running
ROS_CORE_COUNT=`ps -ef | grep "roscore" | grep -v "grep" | wc -l`
if [ ${ROS_CORE_COUNT} -eq 0 ]; then
        echo "please run 'roscore' first"
        exit 1
fi

LOGFILE=rosuds_`date '+%Y%m%d%H%M%S'`.log
LOGDIR=log
TARGETDIR="${SCRIPT_DIR}"/test_cases

if [ ! -d ${LOGDIR} ]; then
        mkdir -p ${LOGDIR}
fi

# setup environment
. ${TARGETDIR}/startenv.sh

for TEST in `ls -1 "${TARGETDIR}"/test_*.sh`
do
        ${TEST} >> ${LOGDIR}/${LOGFILE} 2>&1
done
