# Classification of one accepted-layout raw scan, matched by stamp and frame_id.
# Every source point has one entry, in original row-major order (including rejects).
# These are algorithm decisions, not calibrated confidence or ground-truth labels.
std_msgs/Header header
uint8 SCAN=0
uint8 CLEAR=1
# CLEAR has empty arrays and the last accepted scan's stamp. It invalidates the
# displayed result after rejected/stale input or a clock reset; it is not a scan.
uint8 event
# Doppler fit result for SCAN; clearing cause for CLEAR.
string status
bool fit_valid

uint8 REJECTED=0
uint8 UNCLASSIFIED=1
uint8 STATIC=2
uint8 MOVING=3
uint8 SUSPECTED_GHOST=4

# Quality reasons use first-failure precedence: nonfinite, range, SNR.
# Multiple ghost reasons may be set together. No flags means no rejection.
# GHOST_BEHIND_STATIC may be advisory on MOVING when reject_static_only=false.
# Use classification for the decision, not reason_flags != 0.
uint16 NONFINITE=1
uint16 OUT_OF_RANGE=2
uint16 LOW_SNR=4
uint16 FIT_REJECTED=8
uint16 GHOST_SAME_SPEED=16
uint16 GHOST_DOUBLE_SPEED=32
uint16 GHOST_BEHIND_STATIC=64

uint32[] source_index
uint8[] classification
uint16[] reason_flags
# True only for measured moving_targets within track_radius of a confirmed
# non-ghost track, matching tracked_targets. No predicted points are included.
bool[] track_associated
