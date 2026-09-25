# One entry of a port fault report (see PortFaultReportsMsg).
uint32 module_id
uint32 fault_group
uint32 fault_code
uint32 fault_errno
uint64 fault_time_stamp           # device time stamp of the fault (unit per UIF)
uint32 cycle_count
uint8 instance_id
uint8 criticality
# DEPRECATED misspelling, kept for compatibility; filled with the same value as
# occurrence_count. Will be removed in a future schema version.
uint16 occurence_count
uint16 occurrence_count           # number of occurrences of this fault
