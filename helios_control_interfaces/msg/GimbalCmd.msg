std_msgs/Header header

uint8 gimbal_mode # 0: autoaim 1: cruise 2: debug
uint8 debug_mode  # 0: position 1: velocity

float64 cruise_yaw_vel 
float64 cruise_pitch_vel # pitch velocity is absolute value
float64 yaw
float64 pitch