# Open Drone ID Arm Status
# https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_ARM_STATUS

# [[[cog:
# import mavros_cog
# mavros_cog.idl_decl_enum('MAV_ODID_ARM_STATUS', 'STATUS_')
# ]]]
# MAV_ODID_ARM_STATUS
uint8 STATUS_GOOD_TO_ARM = 0             # Passing arming checks.
uint8 STATUS_PRE_ARM_FAIL_GENERIC = 1    # Generic arming failure, see error string for details.
# [[[end]]] (checksum placeholder)

std_msgs/Header header
uint8 status
string error
