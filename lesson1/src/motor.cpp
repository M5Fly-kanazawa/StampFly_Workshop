//motor.cpp
#include <Arduino.h>
#include "motor.hpp"

void init_motor(void)
{
    //PWM周波数と解像度の設定
    ledcSetup(FRONT_LEFT_MOTOR,  FREQ, RESOLUTION);
    ledcSetup(FRONT_RIGHT_MOTOR, FREQ, RESOLUTION);
    ledcSetup(REAR_LEFT_MOTOR,   FREQ, RESOLUTION);
    ledcSetup(REAR_RIGHT_MOTOR,  FREQ, RESOLUTION);

    ledcAttachPin(PIN_FRONT_LEFT_MOTOR,  FRONT_LEFT_MOTOR);
    ledcAttachPin(PIN_FRONT_RIGHT_MOTOR, FRONT_RIGHT_MOTOR);
    ledcAttachPin(PIN_REAR_LEFT_MOTOR,   REAR_LEFT_MOTOR);
    ledcAttachPin(PIN_REAR_RIGHT_MOTOR,  REAR_RIGHT_MOTOR);
}

void set_motor_duty(uint8_t motor_handle, float duty)
{
    ledcWrite(motor_handle, (uint32_t)(DUTY_MAX*duty));
}
