#! /bin/bash
#wget 'http://infinity.csail.mit.edu/data/2011/2011-01-27-07-49-54.bag'
#mv 2011-01-27-07-49-54.bag test.bag
echo > rosbag_info_log.txt
echo > result.txt
files=$(ls "/media/olga/Expansion Drive" | grep -E ".*\.bag")
chmod +x test.sh
for file in $files
do
echo "$file"
echo "$file" >> rosbag_info_log.txt
#cp "/media/olga/Expansion Drive/$file" test.bag 
sum_of_all=0
test_num=0
params=("path" "size" "version" "duration" "start" "end" "compression" "types" "topics" "messages" "indexed")
for param in "${params[@]}"
do
    test_res=$(./test.sh "$param" "/media/olga/Expansion Drive/$file")
    echo "ratio $test_res" >> rosbag_info_log.txt
    test_with_freq_res=$(./test.sh "$param" "/media/olga/Expansion Drive/$file" --freq)	
    echo "ratio $test_with_freq_res" >> rosbag_info_log.txt
    if [ "$test_res" == "FAIL" ] || [ "$test_with_freq_res" == "FAIL" ];then
         exit 1
    fi
    ((test_num+=2))
    sum_of_all=$(echo "$sum_of_all + $test_res + $test_with_freq_res" | bc -l)
done
echo "average ratio"
echo "scale=2; $sum_of_all / $test_num" | bc -l
#rm test.bag
done
