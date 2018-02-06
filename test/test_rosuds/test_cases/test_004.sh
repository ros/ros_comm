#!/bin/bash

echo ""
echo "TEST UDS 004 ====================="
echo ""
TESTRES=0

set -x

unset ROS_UDS_EXT_ENABLE
${CPP_ADD_TWO_INTS_CLIENT} 1 2 $((SLEEP_TIME+2)) >/dev/null 2>&1  &
SERVICE_CLIENTPID1=$!
sleep ${SLEEP_TIME}

if [ -S "${UDS_STREAM_PATH}${SERVICE_CLIENTPID1}"-* ]; then
        ls -l "${UDS_STREAM_PATH}${SERVICE_CLIENTPID1}"-*
        TESTRES=1
fi

if [ -S "${UDS_DATAGRAM_PATH}${SERVICE_CLIENTPID1}"-* ]; then
        ls -l "${UDS_DATAGRAM_PATH}${SERVICE_CLIENTPID1}"-*
        TESTRES=1
fi

SERVER_STREAM_COUNT=`get_uds_stream_server_count "${SERVICE_CLIENTPID1}"`
if [ ${SERVER_STREAM_COUNT} -ne 0 ]; then
        TESTRES=1
fi

SERVER_DATAGRAM_COUNT=`get_uds_datagram_server_count "${SERVICE_CLIENTPID1}"`
if [ ${SERVER_DATAGRAM_COUNT} -ne 0 ]; then
        TESTRES=1
fi

set +x

kill -INT ${SERVICE_CLIENTPID1}
wait ${SERVICE_CLIENTPID1}

echo ""
if [ ${TESTRES} -eq 0 ]; then
        echo "RESULT: OK"
else
        echo "RESULT: NG"
fi
