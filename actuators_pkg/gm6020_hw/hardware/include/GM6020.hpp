#pragma once

#include <cstdint>
#include <vector>

#define HW_IF_CURRENT "current"
#define HW_IF_ANGLE "angle"
#define HW_IF_SPEED "speed"
#define HW_IF_EFFORT "effort"
#define HW_IF_TEMP "temperature"

typedef struct GM6020Command {
    // head is 0x1ff
    int actuator_current_1;
    int actuator_current_2;
    int actuator_current_3;
    int actuator_current_4;
}GM6020_Cmd;

typedef struct GM6020State {
    // head is 0x204 + number
    // 0-8191
    int angle;
    // rpm
    int speed;
    int effort;
    int temperature;
    // converter funtion
}GM6020_State;

inline double* convert_write_buffer_to_states(std::vector<GM6020_State>& states) {
    static double res[4];
    // int angle, speed, effort, temperature;
    // angle = state->angle_high;
    // angle = angle << 8;
    // angle = angle | state->angle_low;
    // res[0] = angle;
    // speed = state->speed_high;
    // speed = speed << 8;
    // speed = speed | state->speed_low;
    // res[1] = speed;
    // effort = state->effort_high;
    // effort = effort << 8;
    // effort = effort | state->effort_low;
    // res[2] = effort;
    // res[3] = temperature;
    return res;
}

inline void convert_command_to_write_buffer(GM6020_Cmd& cmd, uint8_t* buffer) {
    
}