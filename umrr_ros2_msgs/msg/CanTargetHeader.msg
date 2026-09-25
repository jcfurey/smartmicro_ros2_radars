# Header of a CAN target list, published on smart_radar/can_targetheader_N with
# the same stamp/frame_id as can_targets_N. Units follow the UIF CAN signals.
std_msgs/Header header

float32 cycle_time                # [s] cycle duration (CAN signal CycleDuration)
uint8 number_of_targets
uint32 cycle_count
uint8 acquisition_setup           # opaque acquisition setup feedback
uint32 time_stamp                 # [s] device time stamp, integer part
float32 acq_ts_fraction           # fractional part of the acquisition time stamp
