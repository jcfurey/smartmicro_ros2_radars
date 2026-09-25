# Header of an Ethernet (port-based) object list, published on
# smart_radar/port_objectheader_N with the same stamp/frame_id as port_objects_N.
std_msgs/Header header

# Generic port header.
uint32 port_identifier
int16 port_ver_major
int16 port_ver_minor
uint32 port_size                  # [bytes] including this header
uint8 body_endianness
uint8 port_index
uint8 header_ver_major
uint8 header_ver_minor

# Object list header.
float32 cycle_time                # [s] measurement cycle time
uint16 number_of_objects
uint64 ts_measurement             # device time stamp of the measurement (epoch/unit per UIF)
