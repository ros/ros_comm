#!/bin/bash

echo ""
echo "TEST UDS 034 ====================="
echo ""
TESTRES=0

set -x

# pub exec -> sub exec -> sub exit -> sub exec -> pub exit -> sub exit

# pub exec
ROS_UDS_EXT_ENABLE=on ${CPP_TALKER} >/dev/null 2>&1  &
TALKERPID1=$!
sleep ${SLEEP_TIME}
OLD_SERVER_COUNT=`get_uds_datagram_server_count "${TALKERPID1}"`
OLD_CONNECTED_OUT_COUNT=`get_uds_datagram_connected_out_count "${TALKERPID1}"`
EXPECTED_SERVER_COUNT=$((OLD_SERVER_COUNT))
EXPECTED_CONNECTED_OUT_COUNT=$((OLD_CONNECTED_OUT_COUNT+1))

# sub exec
ROS_UDS_EXT_ENABLE=on ${CPP_LISTENER_UNRELIABLE} >/dev/null 2>&1  &
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

# sub exit
kill -INT ${LISTENERPID1}
wait ${LISTENERPID1}
sleep ${SLEEP_TIME}
EXPECTED_SERVER_COUNT=$((OLD_SERVER_COUNT))
EXPECTED_CONNECTED_OUT_COUNT=$((OLD_CONNECTED_OUT_COUNT))
NEW_SERVER_COUNT=`get_uds_datagram_server_count "${TALKERPID1}"`
NEW_CONNECTED_OUT_COUNT=`get_uds_datagram_connected_out_count "${TALKERPID1}"`

if [ ${NEW_SERVER_COUNT} -ne ${EXPECTED_SERVER_COUNT} ]; then
        TESTRES=1
fi
if [ ${NEW_CONNECTED_OUT_COUNT} -ne ${EXPECTED_CONNECTED_OUT_COUNT} ]; then
        TESTRES=1
fi

# sub exec
ROS_UDS_EXT_ENABLE=on ${CPP_LISTENER_UNRELIABLE} >/dev/null 2>&1  &
LISTENERPID1=$!
sleep ${SLEEP_TIME}
EXPECTED_SERVER_COUNT=$((OLD_SERVER_COUNT))
EXPECTED_CONNECTED_OUT_COUNT=$((OLD_CONNECTED_OUT_COUNT+1))
NEW_SERVER_COUNT=`get_uds_datagram_server_count "${TALKERPID1}"`
NEW_CONNECTED_OUT_COUNT=`get_uds_datagram_connected_out_count "${TALKERPID1}"`

if [ ${NEW_SERVER_COUNT} -ne ${EXPECTED_SERVER_COUNT} ]; then
        TESTRES=1
fi
if [ ${NEW_CONNECTED_OUT_COUNT} -ne ${EXPECTED_CONNECTED_OUT_COUNT} ]; then
        TESTRES=1
fi

# pub exit
kill -INT ${TALKERPID1}
wait ${TALKERPID1}

# sub exit
kill -INT ${LISTENERPID1}
wait ${LISTENERPID1}
sleep ${SLEEP_TIME}

set +x

echo ""
if [ ${TESTRES} -eq 0 ]; then
        echo "RESULT: OK"
else
        echo "RESULT: NG"
fi
