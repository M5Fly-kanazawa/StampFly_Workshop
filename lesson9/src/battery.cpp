#include "battery.hpp"

INA3221 ina3221(INA3221_ADDR40_GND);
float Voltage=0.0; 

void init_battery(void)
{
  Wire1.begin(3, 4,400000UL);
  ina3221.begin(&Wire1);
  INA3221 ina3221(INA3221_ADDR40_GND);
}

float get_voltage(void)
{
   return ina3221.getVoltage(INA3221_CH2);
}
