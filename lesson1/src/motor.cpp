//motor.cpp
#include <Arduino.h>
#include "motor.hpp"

void init_motor(void)
{
  ledcSetup(FRONT_LEFT_MOTOR, FREQ, RESOLUTION);
  ledcSetup(FRONT_RIGHT_MOTOR, FREQ, RESOLUTION);
  ledcSetup(REAR_LEFT_MOTOR, FREQ, RESOLUTION);
  ledcSetup(REAR_RIGHT_MOTOR, FREQ, RESOLUTION);
  ledcAttachPin(pwmFrontLeft, FrontLeft_motor);
  ledcAttachPin(pwmFrontRight, FrontRight_motor);
  ledcAttachPin(pwmRearLeft, RearLeft_motor);
  ledcAttachPin(pwmRearRight, RearRight_motor);
  
  #if 0
  //motor test
  for (uint8_t i=0; i<4; i++)
  {
    ledcWrite(i, 30);
    delay(800);
    ledcWrite(i, 0);
    delay(500);
  }
  #endif
}
