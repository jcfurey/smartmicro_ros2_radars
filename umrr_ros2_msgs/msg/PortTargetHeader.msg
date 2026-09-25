# Header of an Ethernet (port-based) target list, published on
# smart_radar/port_targetheader_N with the same stamp/frame_id as port_targets_N.
# Units follow the vendor user-interface (UIF) serialization definitions.
# Fields a model's interface does not provide are left at zero.
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

# Target list header.
float32 cycle_time                # [s] measurement cycle time
uint16 number_of_targets
uint8 acquisition_tx_ant_idx      # index of the transmit antenna used for acquisition
uint8 acquisition_sweep_idx       # index of the frequency sweep used for acquisition
uint8 acquisition_cf_idx          # index of the centre frequency used for acquisition
uint8 prf                         # PRF setting index (umrr9f v3.2.0: 0..2 first set, 3..5 second)
# Reference time of acquisition_start: 0 = unknown/local, 1 = SMS time sync, 2 = PTP.
uint8 acquisition_time_stamp_base
# DEPRECATED misspelling, kept for compatibility; filled with the same value as
# unambiguous_speed. Will be removed in a future schema version.
float32 umambiguous_speed         # [m/s]
uint64 acquisition_start          # start of acquisition, NTP-coded (see acquisition_time_stamp_base)
# Opaque UMRR-96 acquisition setup word; bit meanings are not yet verified.
# Other model callbacks leave valid=false instead of implying a measured zero.
bool acquisition_setup_valid
uint16 acquisition_setup
float32 unambiguous_speed         # [m/s] unambiguous radial speed interval of this cycle
