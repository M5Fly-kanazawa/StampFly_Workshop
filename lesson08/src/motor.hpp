#ifndef MOTOR_HPP
#define MOTOR_HPP

#define FRONT_LEFT_MOTOR 0
#define FRONT_RIGHT_MOTOR 1
#define REAR_LEFT_MOTOR 2
#define REAR_RIGHT_MOTOR 3

#define PIN_FRONT_LEFT_MOTOR 5 
#define PIN_FRONT_RIGHT_MOTOR 42
#define PIN_REAR_LEFT_MOTOR 10
#define PIN_REAR_RIGHT_MOTOR 41

#define FREQ 150000
#define RESOLUTION 8
//DUTY_MAX = 2^RESOLUTION-1
#define DUTY_MAX 255

void init_motor(void);
void set_motor_duty(uint8_t motor_handle, float duty);
void stop_motor(void);


#endif 