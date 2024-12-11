//Lesson 2
//スロットルレーバーでモータの回転が変わるようにする
#include "flight_control.hpp"
#include "motor.hpp"
#include "rc.hpp"
#include "led.hpp"
#include "imu.hpp"
#include <math.h>

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

//割り込み設定
void init_interrupt(void)
{
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 2500, true);
  timerAlarmEnable(timer);
}

//Initialize Multi copter
void init_copter(void)
{
  //Initialize Serial communication
  USBSerial.begin(115200);
  delay(1000);
  USBSerial.printf("Init StampFly!\r\n");

  //モータ設定
  init_motor();
  //RC設定
  init_rc();
  //LED設定
  init_led(0);
  //IMUの設定
  imu_init();
  imu_update();

  //割り込み設定
  init_interrupt();
  USBSerial.printf("Join StampFly!\r\n");
}

//Main loop
void loop_400Hz(void)
{
  //以下は変更しない
  while(Loop_flag==0);
  Loop_flag = 0;
  //以上は変更しない

  //Start of Loop_400Hz function
  //以下に記述したコードが400Hzで繰り返される
  
  //LED10秒動後に変化する
  if (Loop_counter < 4000)
  {
    board_bottom_led(RED, 1);
    board_tail_led(GREEN, 1);
    stamp_led(BLUE, 1);  
  }
  else
  {
    board_bottom_led(YELLOW, 1);
    board_tail_led(YELLOW, 1);
    stamp_led(YELLOW, 1);      
  }
  
  Loop_counter ++ ;

  //0.01秒ごとにLoop_counterの値と加速度Xと角速度Xを端末に表示
  float ax, gx;
  ax = imu_get_acc_x();
  gx = imu_get_gyro_x();
  if(Loop_counter%40==0) 
    USBSerial.printf("%6d,%6.3f,%6.3f,%6.3f\n\r", Loop_counter, ax, gx, sin((float)Loop_counter*0.0025));
  
  imu_update();
  FastLED.show();
  //End of Loop_400Hz function
}
