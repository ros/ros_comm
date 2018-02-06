#!/bin/bash

echo ""
echo "TEST UDS 035 ====================="
echo ""
TESTRES=0

set -x

unset ROS_UDS_EXT_ENABLE
${PY_TALKER} >/dev/null 2>&1  &
TALKERPID1=$!
sleep ${SLEEP_TIME}

if [ -S "${UDS_STREAM_PATH}${TALKERPID1}"-* ]; then
        ls -l "${UDS_STREAM_PATH}${TALKERPID1}"-*
        TESTRES=1
fi

SERVER_STREAM_COUNT=`get_uds_stream_server_count "${TALKERPID1}"`
if [ ${SERVER_STREAM_COUNT} -ne 0 ]; then
        TESTRES=1
fi

set +x

kill -INT ${TALKERPID1}
wait ${TALKERPID1}

echo ""
if [ ${TESTRES} -eq 0 ]; then
        echo "RESULT: OK"
else
        echo "RESULT: NG"
fi
