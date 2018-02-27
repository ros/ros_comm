#!/bin/bash

# check roscore running
ROS_CORE_COUNT=`ps -ef | grep "roscore" | grep -v "grep" | wc -l`
if [ ${ROS_CORE_COUNT} -eq 0 ]; then
    echo -e "Usage:\n\tplease run roscore first: \"roscore\" for TCPROS/UDPROS and \"ROS_UDS_EXT_ENABLE=on roscore\" for UDSROS"
    exit 1
fi

# check parameter
if [ $# -ne 3 ]; then
    echo -e "Usage:\n\t./run_perf_test.sh test_loop_count subscirber_count transmission_type(choose in UDS and TCP_UDP)"
    exit 1
fi

test_type=$3
size_list="256B 4KB 256KB 2MB 8MB"

if [ ${test_type} = "UDS" ]; then
    for size in ${size_list}
    do
        echo -e "\ntransport benchmark result(${size}):"
        ROS_UDS_EXT_ENABLE=on rosrun test_perf_benchmark talker $1 ${size} $2 &
        sleep 1

        count=1
        while [ ${count} -le $2 ]
        do
            ROS_UDS_EXT_ENABLE=on rosrun test_perf_benchmark listener listener_${size}_${count} &
            count=$(( ${count} + 1 ))
        done

        wait
    done
elif [ ${test_type} = "TCP_UDP" ]; then
    for size in ${size_list}
    do
        echo -e "\ntransport benchmark result(${size}):"
        rosrun test_perf_benchmark talker $1 ${size} $2 &
        sleep 1

        count=1
        while [ ${count} -le $2 ]
        do
            rosrun test_perf_benchmark listener listener_${size}_${count} &
            count=$(( ${count} + 1 ))
        done

        wait
    done
else
    echo -e "Usage:\n\t./run_perf_test.sh test_loop_count subscirber_count transmission_type(choose in UDS and TCP_UDP)"
    exit 1
fi

wait
