std_msgs/Header header

uint8 gimbal_mode # 0: autoaim 1: cruise 

float64 yaw_value     # when in autoaim mode, the angle of yaw and pitch is under imu frame
float64 pitch_value   # when in cruise mode, pitch velocity is absolute value
