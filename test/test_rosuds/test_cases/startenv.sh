#!/bin/bash

export UDS_STREAM_PATH="/tmp/ros-uds-stream-"
export UDS_DATAGRAM_PATH="/tmp/ros-uds-datagram-"

# get server count at server if using stream
get_uds_stream_server_count() {
        set +x
        local pid=$1
        local count=`netstat -anp 2>/dev/null | grep " ${pid}/" | grep "STREAM" | grep "LISTENING" | grep "${UDS_STREAM_PATH}" | wc -l`
        echo ${count}
        set -x
}

# get client connected count at server if using stream.
get_uds_stream_connected_in_count() {
        set +x
        local pid=$1
        local count=`netstat -anp 2>/dev/null | grep " ${pid}/" | grep "STREAM" | grep "CONNECTED" | grep "${UDS_STREAM_PATH}" | wc -l`
        echo ${count}
        set -x
}

# get connected count which was from client to server at client if using stream
get_uds_stream_connected_out_count() {
        set +x
        local pid=$1
        local count=`netstat -anp 2>/dev/null | grep " ${pid}/" | grep "STREAM" | grep "CONNECTED" | grep -v "${UDS_STREAM_PATH}" | wc -l`
        echo ${count}
        set -x
}
export -f get_uds_stream_server_count
export -f get_uds_stream_connected_in_count
export -f get_uds_stream_connected_out_count

# get server count at server if using datagram
get_uds_datagram_server_count() {
        set +x
        local pid=$1
        local count=`netstat -anp 2>/dev/null | grep " ${pid}/" | grep "DGRAM" | grep "${UDS_DATAGRAM_PATH}" | wc -l`
        echo ${count}
        set -x
}

# get connected count which was from client to server at client if using datagram
get_uds_datagram_connected_out_count() {
        set +x
        local pid=$1
        local count=`netstat -anp 2>/dev/null | grep " ${pid}/" | grep "DGRAM" | grep -v "${UDS_DATAGRAM_PATH}" | wc -l`
        echo ${count}
        set -x
}
export -f get_uds_datagram_server_count
export -f get_uds_datagram_connected_out_count

sleep_time=3
if [ -f /etc/lsb-integrator ]; then
    ROSUDS_TEST_PLATFORM=`grep 'DISTRIB_CODENAME' /etc/lsb-integrator | awk -F'=' '{print $2}'`
    if [ "$ROSUDS_TEST_PLATFORM" = "BDK16-arm64" ]; then
        sleep_time=5
    fi
fi
export SLEEP_TIME=${sleep_time}
