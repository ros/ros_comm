#!/bin/bash

echo ""
echo "TEST UDS 038 ====================="
echo ""
TESTRES=0

set -x

${PY_TALKER} >/dev/null 2>&1  &
TALKERPID1=$!
sleep ${SLEEP_TIME}

ls -l "${UDS_STREAM_PATH}${TALKERPID1}"-*
if [ ! -S "${UDS_STREAM_PATH}${TALKERPID1}"-* ]; then
        TESTRES=1
fi

SERVER_STREAM_COUNT=`get_uds_stream_server_count "${TALKERPID1}"`
if [ ${SERVER_STREAM_COUNT} -ne 1 ]; then
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