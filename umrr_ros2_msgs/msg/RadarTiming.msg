# Header stamp is ROS receive time at the SDK callback, not acquisition time.
# Match it to the associated cloud/header or fault report by stamp and frame_id.
std_msgs/Header header
uint32 sensor_id
# Original SDK timestamp in microseconds. Clock epoch/synchronization is unverified.
uint64 device_timestamp_us
uint8 TARGETS=0
uint8 OBJECTS=1
uint8 FAULTS=2
uint8 stream
uint8 ROS_RECEIVE_TIME=0
uint8 timestamp_source
