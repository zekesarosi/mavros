# EKF status report from autopilot
#
# https://mavlink.io/en/messages/ardupilotmega.html#EKF_STATUS_REPORT

uint16 EKF_ATTITUDE = 1
uint16 EKF_VELOCITY_HORIZ = 2
uint16 EKF_VELOCITY_VERT = 4
uint16 EKF_POS_HORIZ_REL = 8
uint16 EKF_POS_HORIZ_ABS = 16
uint16 EKF_POS_VERT_ABS = 32
uint16 EKF_POS_VERT_AGL = 64
uint16 EKF_CONST_POS_MODE = 128
uint16 EKF_PRED_POS_HORIZ_REL = 256
uint16 EKF_PRED_POS_HORIZ_ABS = 512
uint16 EKF_UNINITIALIZED = 1024
uint16 EKF_GPS_GLITCHING = 32768

std_msgs/Header header

uint16 flags

float32 velocity_variance
float32 pos_horiz_variance
float32 pos_vert_variance
float32 compass_variance
float32 terrain_alt_variance
float32 airspeed_variance
