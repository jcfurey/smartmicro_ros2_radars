# Uninterpreted SDK values for UMRR-96 Ethernet ComTargetList.
# Match header stamp/frame_id to port_targets_N. Arrays follow raw cloud order.
# These may contain firmware defaults. Zero does NOT establish perfect confidence,
# valid measurements, or an absence of ambiguity. Do not gate detections with them
# until field validity and the flag bit definitions are verified for the firmware.
std_msgs/Header header
uint32 sensor_id
uint8 SEMANTICS_UNVERIFIED=0
uint8 semantics
float32[] false_alarm_probability_raw
uint32[] flags_raw
