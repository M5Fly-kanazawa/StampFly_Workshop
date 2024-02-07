#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <Arduino.h>

//グローバル関数の宣言
void init_copter(void);
void loop_400Hz(void);

extern volatile uint8_t Loop_flag;

#endif
