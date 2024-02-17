#include "pid.hpp"

#define SMAX 30000.0

pidpara_t roll, pitch, yaw;
float h =0.0025;

float pid_P(float ref, float rate , uint8_t flag)
{
    float err = ref - rate;
    if (flag )roll.s = roll.s + err * h;
    if (roll.s > SMAX) roll.s = SMAX;
    else if (roll.s < -SMAX) roll.s = -SMAX;
    float diff = (err - roll.err)/h;
    roll.err = err;
    return roll.kp*(err + diff * roll.td + roll.s/roll.ti);
}

float pid_Q(float ref, float rate, uint8_t flag)
{
    float err = ref - rate;
    if (flag ) pitch.s = pitch.s + err * h;
    if (pitch.s > SMAX) pitch.s = SMAX;
    else if (pitch.s < -SMAX) pitch.s = -SMAX;
    float diff = (err - pitch.err)/h;
    pitch.err = err;
    return pitch.kp*(err + diff * pitch.td + pitch.s/pitch.ti);    
}

float pid_R(float ref, float rate, uint8_t flag)
{
    float err = ref - rate;
    if (flag )yaw.s = yaw.s + err * h;
    if (yaw.s > SMAX) yaw.s = SMAX;
    else if (yaw.s < -SMAX) yaw.s = -SMAX;
    float diff = (err - yaw.err)/h;
    yaw.err = err;
    return yaw.kp*(err + diff * yaw.td + yaw.s/yaw.ti);        
}


void reset_pid(void)
{
    roll.kp = 0.6;
    roll.ti = 0.7;
    roll.td = 0.01;;
    roll.s = 0.0;

    pitch.kp = 0.75;
    pitch.ti = 0.7;
    pitch.td = 0.025;
    pitch.s = 0.0;

    yaw.kp = 3.0;
    yaw.ti = 0.8;
    yaw.td = 0.01;
    yaw.s = 0.0;
}