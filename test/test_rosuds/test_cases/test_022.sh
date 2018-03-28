#!/bin/bash

echo ""
echo "TEST UDS 022 ====================="
echo ""
TESTRES=0

set -x

${CPP_LISTENER_UNRELIABLE} >/dev/null 2>&1  &
LISTENERPID1=$!
sleep ${SLEEP_TIME}
OLD_SERVER_COUNT=`get_uds_datagram_server_count "${LISTENERPID1}"`
OLD_CONNECTED_OUT_COUNT=`get_uds_datagram_connected_out_count "${LISTENERPID1}"`
EXPECTED_SERVER_COUNT=$((OLD_SERVER_COUNT+2))
EXPECTED_CONNECTED_OUT_COUNT=$((OLD_CONNECTED_OUT_COUNT))

${CPP_TALKER} __name:=t1 >/dev/null 2>&1  &
TALKERPID1=$!
${CPP_TALKER} __name:=t2 >/dev/null 2>&1  &
TALKERPID2=$!
sleep ${SLEEP_TIME}
NEW_SERVER_COUNT=`get_uds_datagram_server_count "${LISTENERPID1}"`
NEW_CONNECTED_OUT_COUNT=`get_uds_datagram_connected_out_count "${LISTENERPID1}"`

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
kill -INT ${TALKERPID2}
wait ${TALKERPID2}
kill -INT ${LISTENERPID1}
wait ${LISTENERPID1}

echo ""
if [ ${TESTRES} -eq 0 ]; then
        echo "RESULT: OK"
else
        echo "RESULT: NG"
fi
