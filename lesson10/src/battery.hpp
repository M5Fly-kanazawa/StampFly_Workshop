#ifndef BATTERY_HPP
#define BATTERY_HPP

#include <Arduino.h>
#include <INA3221.h>

void init_battery(void);
float get_voltage(void);


#endif