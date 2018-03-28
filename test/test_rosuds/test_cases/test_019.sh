#!/bin/bash

echo ""
echo "TEST UDS 019 ====================="
echo ""
TESTRES=0

set -x

${CPP_TALKER} >/dev/null 2>&1  &
TALKERPID1=$!
sleep ${SLEEP_TIME}
OLD_SERVER_COUNT=`get_uds_datagram_server_count "${TALKERPID1}"`
OLD_CONNECTED_OUT_COUNT=`get_uds_datagram_connected_out_count "${TALKERPID1}"`
EXPECTED_SERVER_COUNT=$((OLD_SERVER_COUNT))
EXPECTED_CONNECTED_OUT_COUNT=$((OLD_CONNECTED_OUT_COUNT+10))

${CPP_LISTENER_UNRELIABLE} __name:=l1 >/dev/null 2>&1  &
LISTENERPID1=$!
${CPP_LISTENER_UNRELIABLE} __name:=l2 >/dev/null 2>&1  &
LISTENERPID2=$!
${CPP_LISTENER_UNRELIABLE} __name:=l3 >/dev/null 2>&1  &
LISTENERPID3=$!
${CPP_LISTENER_UNRELIABLE} __name:=l4 >/dev/null 2>&1  &
LISTENERPID4=$!
${CPP_LISTENER_UNRELIABLE} __name:=l5 >/dev/null 2>&1  &
LISTENERPID5=$!
${CPP_LISTENER_UNRELIABLE} __name:=l6 >/dev/null 2>&1  &
LISTENERPID6=$!
${CPP_LISTENER_UNRELIABLE} __name:=l7 >/dev/null 2>&1  &
LISTENERPID7=$!
${CPP_LISTENER_UNRELIABLE} __name:=l8 >/dev/null 2>&1  &
LISTENERPID8=$!
${CPP_LISTENER_UNRELIABLE} __name:=l9 >/dev/null 2>&1  &
LISTENERPID9=$!
${CPP_LISTENER_UNRELIABLE} __name:=l10 >/dev/null 2>&1  &
LISTENERPID10=$!
sleep $((SLEEP_TIME*3))
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
kill -INT ${LISTENERPID2}
wait ${LISTENERPID2}
kill -INT ${LISTENERPID3}
wait ${LISTENERPID3}
kill -INT ${LISTENERPID4}
wait ${LISTENERPID4}
kill -INT ${LISTENERPID5}
wait ${LISTENERPID5}
kill -INT ${LISTENERPID6}
wait ${LISTENERPID6}
kill -INT ${LISTENERPID7}
wait ${LISTENERPID7}
kill -INT ${LISTENERPID8}
wait ${LISTENERPID8}
kill -INT ${LISTENERPID9}
wait ${LISTENERPID9}
kill -INT ${LISTENERPID10}
wait ${LISTENERPID10}

echo ""
if [ ${TESTRES} -eq 0 ]; then
        echo "RESULT: OK"
else
        echo "RESULT: NG"
fi
