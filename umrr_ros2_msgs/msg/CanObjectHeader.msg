# Header of a CAN object list, published on smart_radar/can_objectheader_N with
# the same stamp/frame_id as can_objects_N. Units follow the UIF CAN signals.
std_msgs/Header header

float32 cycle_time                # [s] cycle duration (CAN signal CycleDuration)
uint16 number_of_objects
float32 ego_speed                 # [km/h] ego speed as received by the sensor (not m/s)
float32 ego_speed_quality         # quality indicator, unitless
float32 ego_yaw_rate              # [rad/s] ego yaw rate as received by the sensor
float32 ego_yaw_rate_quality      # quality indicator, unitless
uint32 cycle_count
uint8 dyn_source                  # source of the ego dynamics (UIF enumeration)
