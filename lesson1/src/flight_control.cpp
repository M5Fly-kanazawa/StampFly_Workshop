//Lesson 1
#include "flight_control.hpp"
#include "motor.hpp"

//Global variable
const float Control_period = 0.0025f;//400Hz //制御周期
volatile uint8_t Loop_flag = 0;
uint32_t Loop_counter = 0;

//割り込み関数
//Intrupt function
hw_timer_t * timer = NULL;
void IRAM_ATTR onTimer() 
{
  Loop_flag = 1;
}

//Initialize Multi copter
void init_copter(void)
{
  //Initialize Serial communication
  USBSerial.begin(115200);
  delay(1000);
  USBSerial.printf("Start StampFly!\r\n");
  
  //Initialize Motor
  init_motor();

  //割り込み設定
  //Initialize intrupt
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 2500, true);
  timerAlarmEnable(timer);
  USBSerial.printf("Finish StampFly init!\r\n");
}

//Main loop
void loop_400Hz(void)
{
  while(Loop_flag==0);
  Loop_flag = 0;
  
  //Start of Loop_400Hz function
  
  if (Loop_counter < 800)
  {
    //Start motor
    set_motor_duty(FRONT_LEFT_MOTOR,  0.15);
    set_motor_duty(FRONT_RIGHT_MOTOR, 0.15);
    set_motor_duty(REAR_LEFT_MOTOR,   0.15);
    set_motor_duty(REAR_RIGHT_MOTOR,  0.15);
  }
  else if (Loop_counter < 1600)
  {
    stop_motor();
  }
  else Loop_counter = 0;

  Loop_counter ++ ;
  if(Loop_counter%400==0) USBSerial.printf("%d\r\n", Loop_counter);
  

  //End of Loop_400Hz function
}
