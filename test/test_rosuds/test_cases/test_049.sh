#!/bin/bash

echo ""
echo "TEST UDS 049 ====================="
echo ""
TESTRES=0

set -x

${PY_ADD_TWO_INTS_SERVER} $((SLEEP_TIME+2)) >/dev/null 2>&1  &
SERVICE_SERVERPID1=$!
sleep ${SLEEP_TIME}

ls -l "${UDS_STREAM_PATH}${SERVICE_SERVERPID1}"-*
if [ ! -S "${UDS_STREAM_PATH}${SERVICE_SERVERPID1}"-* ]; then
        TESTRES=1
fi

OLD_SERVER_COUNT=`get_uds_stream_server_count "${SERVICE_SERVERPID1}"`
OLD_CONNECTED_IN_COUNT=`get_uds_stream_connected_in_count "${SERVICE_SERVERPID1}"`
EXPECTED_SERVER_COUNT=$((OLD_SERVER_COUNT))
EXPECTED_CONNECTED_IN_COUNT=$((OLD_CONNECTED_IN_COUNT+1))

${PY_ADD_TWO_INTS_CLIENT} 1 2 >/dev/null 2>&1  &
SERVICE_CLIENTPID1=$!
sleep ${SLEEP_TIME}

NEW_SERVER_COUNT=`get_uds_stream_server_count "${SERVICE_SERVERPID1}"`
NEW_CONNECTED_IN_COUNT=`get_uds_stream_connected_in_count "${SERVICE_SERVERPID1}"`

if [ ${NEW_SERVER_COUNT} -ne ${EXPECTED_SERVER_COUNT} ]; then
        TESTRES=1
fi
if [ ${NEW_CONNECTED_IN_COUNT} -ne ${EXPECTED_CONNECTED_IN_COUNT} ]; then
        TESTRES=1
fi

set +x

kill -INT ${SERVICE_SERVERPID1}
wait ${SERVICE_SERVERPID1}

echo ""
if [ ${TESTRES} -eq 0 ]; then
        echo "RESULT: OK"
else
        echo "RESULT: NG"
fi
