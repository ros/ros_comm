#!/bin/bash

echo ""
echo "TEST UDS 031 ====================="
echo ""
TESTRES=0

set -x

# sub exec -> pub exec -> pub exit -> pub exec -> sub exit -> pub exit

# sub exec
ROS_UDS_EXT_ENABLE=on rosrun test_rosuds listener_unreliable_uds >/dev/null 2>&1  &
LISTENERPID1=$!
sleep ${SLEEP_TIME}
OLD_SERVER_COUNT=`get_uds_datagram_server_count "${LISTENERPID1}"`
OLD_CONNECTED_OUT_COUNT=`get_uds_datagram_connected_out_count "${LISTENERPID1}"`
EXPECTED_SERVER_COUNT=$((OLD_SERVER_COUNT+1))
EXPECTED_CONNECTED_OUT_COUNT=$((OLD_CONNECTED_OUT_COUNT))

# pub exec
ROS_UDS_EXT_ENABLE=on rosrun test_rosuds talker_uds >/dev/null 2>&1  &
TALKERPID1=$!
sleep ${SLEEP_TIME}
NEW_SERVER_COUNT=`get_uds_datagram_server_count "${LISTENERPID1}"`
NEW_CONNECTED_OUT_COUNT=`get_uds_datagram_connected_out_count "${LISTENERPID1}"`

if [ ${NEW_SERVER_COUNT} -ne ${EXPECTED_SERVER_COUNT} ]; then
        TESTRES=1
fi
if [ ${NEW_CONNECTED_OUT_COUNT} -ne ${EXPECTED_CONNECTED_OUT_COUNT} ]; then
        TESTRES=1
fi

# pub exit
kill -INT ${TALKERPID1}
wait ${TALKERPID1}
sleep ${SLEEP_TIME}
EXPECTED_SERVER_COUNT=$((OLD_SERVER_COUNT))
EXPECTED_CONNECTED_OUT_COUNT=$((OLD_CONNECTED_OUT_COUNT))
NEW_SERVER_COUNT=`get_uds_datagram_server_count "${LISTENERPID1}"`
NEW_CONNECTED_OUT_COUNT=`get_uds_datagram_connected_out_count "${LISTENERPID1}"`

if [ ${NEW_SERVER_COUNT} -ne ${EXPECTED_SERVER_COUNT} ]; then
        TESTRES=1
fi
if [ ${NEW_CONNECTED_OUT_COUNT} -ne ${EXPECTED_CONNECTED_OUT_COUNT} ]; then
        TESTRES=1
fi

# pub exec
ROS_UDS_EXT_ENABLE=on rosrun test_rosuds talker_uds >/dev/null 2>&1  &
TALKERPID1=$!
sleep ${SLEEP_TIME}
EXPECTED_SERVER_COUNT=$((OLD_SERVER_COUNT+1))
EXPECTED_CONNECTED_OUT_COUNT=$((OLD_CONNECTED_OUT_COUNT))
NEW_SERVER_COUNT=`get_uds_datagram_server_count "${LISTENERPID1}"`
NEW_CONNECTED_OUT_COUNT=`get_uds_datagram_connected_out_count "${LISTENERPID1}"`

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

set +x

echo ""
if [ ${TESTRES} -eq 0 ]; then
        echo "RESULT: OK"
else
        echo "RESULT: NG"
fi
