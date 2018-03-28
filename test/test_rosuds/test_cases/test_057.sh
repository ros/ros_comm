#!/bin/bash

echo ""
echo "TEST UDS 057 ====================="
echo ""
TESTRES=0

set -x

${PY_TALKER} >/dev/null 2>&1  &
TALKERPID1=$!
sleep ${SLEEP_TIME}
OLD_SERVER_COUNT=`get_uds_stream_server_count "${TALKERPID1}"`
OLD_CONNECTED_IN_COUNT=`get_uds_stream_connected_in_count "${TALKERPID1}"`
EXPECTED_SERVER_COUNT=$((OLD_SERVER_COUNT))
EXPECTED_CONNECTED_IN_COUNT=$((OLD_CONNECTED_IN_COUNT+2))

${CPP_LISTENER} __name:=no1 >/dev/null 2>&1  &
LISTENERPID1=$!
${PY_LISTENER} __name:=no2 >/dev/null 2>&1  &
LISTENERPID2=$!
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
kill -INT ${LISTENERPID2}
wait ${LISTENERPID2}

echo ""
if [ ${TESTRES} -eq 0 ]; then
        echo "RESULT: OK"
else
        echo "RESULT: NG"
fi
