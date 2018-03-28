#!/bin/bash

echo ""
echo "TEST UDS 017 ====================="
echo ""
TESTRES=0

set -x

${CPP_TALKER} >/dev/null 2>&1  &
TALKERPID1=$!
sleep ${SLEEP_TIME}
OLD_SERVER_COUNT=`get_uds_datagram_server_count "${TALKERPID1}"`
OLD_CONNECTED_OUT_COUNT=`get_uds_datagram_connected_out_count "${TALKERPID1}"`
EXPECTED_SERVER_COUNT=$((OLD_SERVER_COUNT))
EXPECTED_CONNECTED_OUT_COUNT=$((OLD_CONNECTED_OUT_COUNT+1))

${CPP_LISTENER_UNRELIABLE} >/dev/null 2>&1  &
LISTENERPID1=$!
sleep ${SLEEP_TIME}
NEW_SERVER_COUNT=`get_uds_datagram_server_count "${TALKERPID1}"`
NEW_CONNECTED_OUT_COUNT=`get_uds_datagram_connected_out_count "${TALKERPID1}"`

if [ ${NEW_SERVER_COUNT} -ne ${EXPECTED_SERVER_COUNT} ]; then
        TESTRES=1
fi
if [ ${NEW_CONNECTED_OUT_COUNT} -ne ${EXPECTED_CONNECTED_OUT_COUNT} ]; then
        TESTRES=1
fi

set +x

sleep ${SLEEP_TIME}

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
