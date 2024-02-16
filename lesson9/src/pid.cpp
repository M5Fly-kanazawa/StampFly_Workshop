//pid.cpp
#include "pid.hpp"

#define SMAX 10000.0

pidpara_t roll, pitch, yaw;
float h =0.0025;

float pid_P(float ref, float rate)
{
    float err = ref - rate;
    roll.s = roll.s + err * h;
    if (roll.s > SMAX) roll.s = SMAX;
    else if (roll.s < -SMAX) roll.s = -SMAX;
    float diff = (err - roll.err)/h;
    roll.err = err;
    return roll.kp*(1 + diff * roll.td + roll.s/roll.ti);
}

float pid_Q(float ref, float rate)
{
    float err = ref - rate;
    pitch.s = pitch.s + err * h;
    if (pitch.s > SMAX) pitch.s = SMAX;
    else if (pitch.s < -SMAX) pitch.s = -SMAX;
    float diff = (err - pitch.err)/h;
    pitch.err = err;
    return pitch.kp*(1 + diff * pitch.td + pitch.s/pitch.ti);    
}

float pid_R(float ref, float rate)
{
    float err = ref - rate;
    yaw.s = yaw.s + err * h;
    if (yaw.s > SMAX) yaw.s = SMAX;
    else if (yaw.s < -SMAX) yaw.s = -SMAX;
    float diff = (err - yaw.err)/h;
    yaw.err = err;
    return yaw.kp*(1 + diff * yaw.td + yaw.s/yaw.ti);        
}


void reset_pid(void)
{
    roll.kp = 8;
    roll.ti = 0.8;
    roll.td = 0.08;
    roll.s = 0.0;

    pitch.kp = 8;
    pitch.ti = 0.8;
    pitch.td = 0.08;
    pitch.s = 0.0;

    yaw.kp = 8;
    yaw.ti = 0.8;
    yaw.td = 0.08;
    yaw.s = 0.0;
}