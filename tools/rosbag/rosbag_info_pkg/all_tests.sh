#! /bin/bash
wget 'http://infinity.csail.mit.edu/data/2011/2011-01-21-09-01-36.bag'
echo > rosbag_info_log.txt
file=$(ls | grep -E ".*\.bag")
chmod +x test.sh
echo "$file" >> rosbag_info_log.txt
sum_of_all=0
test_num=0
params=("path" "size" "version" "duration" "start" "end" "compression" "types" "topics" "messages" "indexed")
for param in "${params[@]}"
do
    test_res=$(./test.sh "$param" "$file")
    echo "ratio $test_res" >> rosbag_info_log.txt
    test_with_freq_res=$(./test.sh "$param" "$file" --freq)	
    echo "ratio $test_with_freq_res" >> rosbag_info_log.txt
    if [ "$test_res" == "FAIL" ] || [ "$test_with_freq_res" == "FAIL" ];then
         exit 1
    fi
    ((test_num+=2))
    sum_of_all=$(echo "$sum_of_all + $test_res + $test_with_freq_res" | bc -l)
done
echo "average ratio"
echo "scale=2; $sum_of_all / $test_num" | bc -l

