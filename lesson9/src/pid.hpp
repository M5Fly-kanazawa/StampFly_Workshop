#ifndef PID_HPP
#define PID_HPP
//Start of Header file
#include <stdint.h>

typedef struct 
{
    float kp;
    float ti;
    float td;
    float s;
    float err;
} pidpara_t;

void reset_pid(void);

float pid_P(float ref, float rate, uint8_t flag);
float pid_Q(float ref, float rate, uint8_t flag);
float pid_R(float ref, float rate, uint8_t flag);

//END of Header file
#endif