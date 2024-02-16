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
} pid_t;

void pid_control(float p, float q, float r, 
                 float p_ref, float q_ref, float r_ref,
                 float *pu, float *qu, float *ru);

void reset_pid(pid_t *pidst);

//END of Header file
#endif