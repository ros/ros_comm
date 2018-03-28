#!/bin/bash

echo ""
echo "TEST UDS 009 ====================="
echo ""
TESTRES=0

set -x

${CPP_TALKER} >/dev/null 2>&1  &
TALKERPID1=$!
sleep ${SLEEP_TIME}
OLD_SERVER_COUNT=`get_uds_stream_server_count "${TALKERPID1}"`
OLD_CONNECTED_IN_COUNT=`get_uds_stream_connected_in_count "${TALKERPID1}"`
EXPECTED_SERVER_COUNT=$((OLD_SERVER_COUNT))
EXPECTED_CONNECTED_IN_COUNT=$((OLD_CONNECTED_IN_COUNT+1))

${CPP_LISTENER} >/dev/null 2>&1  &
LISTENERPID1=$!
sleep ${SLEEP_TIME}
NEW_SERVER_COUNT=`get_uds_stream_server_count "${TALKERPID1}"`
NEW_CONNECTED_IN_COUNT=`get_uds_stream_connected_in_count "${TALKERPID1}"`

if [ ${NEW_SERVER_COUNT} -ne ${EXPECTED_SERVER_COUNT} ]; then
        TESTRES=1
fi
if [ ${NEW_CONNECTED_IN_COUNT} -ne ${EXPECTED_CONNECTED_IN_COUNT} ]; then
        TESTRES=1
fi

set +x

kill -INT ${TALKERPID1}
wait ${TALKERPID1}
kill -INT ${LISTENERPID1}
wait ${LISTENERPID1}

echo ""
if [ ${TESTRES} -eq 0 ]; then
        echo "RESULT: OK"
else
        echo "RESULT: NG"
fi
