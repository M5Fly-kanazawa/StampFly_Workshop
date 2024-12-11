#include <Arduino.h>
#include "flight_control.hpp"


void setup() {  
  init_copter();
  delay(100);
}

void loop() {
  loop_400Hz();
}
