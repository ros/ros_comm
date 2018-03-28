#!/bin/bash

echo ""
echo "TEST UDS 015 ====================="
echo ""
TESTRES=0

set -x

${CPP_LISTENER} >/dev/null 2>&1  &
LISTENERPID1=$!
sleep ${SLEEP_TIME}
OLD_SERVER_COUNT=`get_uds_stream_server_count "${LISTENERPID1}"`
OLD_CONNECTED_OUT_COUNT=`get_uds_stream_connected_out_count "${LISTENERPID1}"`
EXPECTED_SERVER_COUNT=$((OLD_SERVER_COUNT))
EXPECTED_CONNECTED_OUT_COUNT=$((OLD_CONNECTED_OUT_COUNT+10))

${CPP_TALKER} __name:=no1 >/dev/null 2>&1  &
TALKERPID1=$!
${CPP_TALKER} __name:=no2 >/dev/null 2>&1  &
TALKERPID2=$!
${CPP_TALKER} __name:=no3 >/dev/null 2>&1  &
TALKERPID3=$!
${CPP_TALKER} __name:=no4 >/dev/null 2>&1  &
TALKERPID4=$!
${CPP_TALKER} __name:=no5 >/dev/null 2>&1  &
TALKERPID5=$!
${CPP_TALKER} __name:=no6 >/dev/null 2>&1  &
TALKERPID6=$!
${CPP_TALKER} __name:=no7 >/dev/null 2>&1  &
TALKERPID7=$!
${CPP_TALKER} __name:=no8 >/dev/null 2>&1  &
TALKERPID8=$!
${CPP_TALKER} __name:=no9 >/dev/null 2>&1  &
TALKERPID9=$!
${CPP_TALKER} __name:=no10 >/dev/null 2>&1  &
TALKERPID10=$!
sleep $((SLEEP_TIME*3))
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
kill -INT ${TALKERPID3}
wait ${TALKERPID3}
kill -INT ${TALKERPID4}
wait ${TALKERPID4}
kill -INT ${TALKERPID5}
wait ${TALKERPID5}
kill -INT ${TALKERPID6}
wait ${TALKERPID6}
kill -INT ${TALKERPID7}
wait ${TALKERPID7}
kill -INT ${TALKERPID8}
wait ${TALKERPID8}
kill -INT ${TALKERPID9}
wait ${TALKERPID9}
kill -INT ${TALKERPID10}
wait ${TALKERPID10}

echo ""
if [ ${TESTRES} -eq 0 ]; then
        echo "RESULT: OK"
else
        echo "RESULT: NG"
fi
