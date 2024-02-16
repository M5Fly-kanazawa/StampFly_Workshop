//pid.hpp
#ifndef PID_HPP
#define PID_HPP
//Start of Header file

typedef struct 
{
    float kp;
    float ti;
    float td;
    float s;
} hoge;

void pid_control(void);

void reset_pid(void);

//END of Header file
#endif