#!/bin/bash

echo ""
echo "TEST UDS 006 ====================="
echo ""
TESTRES=0

set -x

ROS_UDS_EXT_ENABLE=on rosrun test_rosuds listener_uds >/dev/null 2>&1  &
LISTENERPID1=$!
sleep ${SLEEP_TIME}

ls -l "${UDS_STREAM_PATH}${LISTENERPID1}"-*
if [ ! -S "${UDS_STREAM_PATH}${LISTENERPID1}"-* ]; then
        TESTRES=1
fi

ls -l "${UDS_DATAGRAM_PATH}${LISTENERPID1}"-*
if [ ! -S "${UDS_DATAGRAM_PATH}${LISTENERPID1}"-* ]; then
        TESTRES=1
fi

SERVER_STREAM_COUNT=`get_uds_stream_server_count "${LISTENERPID1}"`
if [ ${SERVER_STREAM_COUNT} -ne 1 ]; then
        TESTRES=1
fi

SERVER_DATAGRAM_COUNT=`get_uds_datagram_server_count "${LISTENERPID1}"`
if [ ${SERVER_DATAGRAM_COUNT} -ne 1 ]; then
        TESTRES=1
fi

set +x

kill -INT ${LISTENERPID1}
wait ${LISTENERPID1}

echo ""
if [ ${TESTRES} -eq 0 ]; then
        echo "RESULT: OK"
else
        echo "RESULT: NG"
fi
