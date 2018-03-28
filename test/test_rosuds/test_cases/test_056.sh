#!/bin/bash

echo ""
echo "TEST UDS 056 ====================="
echo ""
TESTRES=0

set -x

${CPP_LISTENER} >/dev/null 2>&1  &
LISTENERPID1=$!
sleep ${SLEEP_TIME}
OLD_SERVER_COUNT=`get_uds_stream_server_count "${LISTENERPID1}"`
OLD_CONNECTED_OUT_COUNT=`get_uds_stream_connected_out_count "${LISTENERPID1}"`
EXPECTED_SERVER_COUNT=$((OLD_SERVER_COUNT))
EXPECTED_CONNECTED_OUT_COUNT=$((OLD_CONNECTED_OUT_COUNT+2))

${CPP_TALKER} __name:=no1 >/dev/null 2>&1  &
TALKERPID1=$!
${PY_TALKER} __name:=no2 >/dev/null 2>&1  &
TALKERPID2=$!
sleep ${SLEEP_TIME}
NEW_SERVER_COUNT=`get_uds_stream_server_count "${LISTENERPID1}"`
NEW_CONNECTED_OUT_COUNT=`get_uds_stream_connected_out_count "${LISTENERPID1}"`

if [ ${NEW_SERVER_COUNT} -ne ${EXPECTED_SERVER_COUNT} ]; then
        TESTRES=1
fi
if [ ${NEW_CONNECTED_OUT_COUNT} -ne ${EXPECTED_CONNECTED_OUT_COUNT} ]; then
        TESTRES=1
fi

set +x

kill -INT ${LISTENERPID1}
wait ${LISTENERPID1}
kill -INT ${TALKERPID1}
wait ${TALKERPID1}
kill -INT ${TALKERPID2}
wait ${TALKERPID2}

echo ""
if [ ${TESTRES} -eq 0 ]; then
        echo "RESULT: OK"
else
        echo "RESULT: NG"
fi
