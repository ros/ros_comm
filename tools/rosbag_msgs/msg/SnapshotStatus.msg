# Describes the current status of a running 'rosbag snapshot' node

# Information about the buffer of subscribed topics
# The period, stamp, node_pub, and dropped_msgs fields are left empty
rosgraph_msgs/TopicStatistics[] topics
# If true, new messages are being added to the snapshot buffers
# If false, snapshoter is currently paused or writing to a bag file
bool enabled
