std_msgs/Header header
int8 shooter_mode # 0: stop, 1: high_velocity
int8 dial_mode    # 0: stop, 1: clockwise, 2: counter-clockwise 3: dead
int8 dial_velocity_level # 0-10, larger, the faster
int8 fire_flag  # 0: stop, 1: fire once(int counter-clockwise)