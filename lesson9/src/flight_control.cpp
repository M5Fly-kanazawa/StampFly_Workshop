//Lesson 2
//スロットルレーバーでモータの回転が変わるようにする
#include "flight_control.hpp"
#include "motor.hpp"
#include "rc.hpp"
#include "led.hpp"
#include "imu.hpp"
#include <math.h>
#include "pid.hpp"

//Global variable
const float Control_period = 0.0025f;//400Hz //制御周期
volatile uint8_t Loop_flag = 0;
uint32_t Loop_counter = 0;

//ここから新しくかいたよ
uint8_t Mode = 0;
float Ref_p, Ref_q, Ref_r; 
float RateP, RateQ, RateR;
float DutyFR, DutyRR, DutyRL, DutyFL;

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

  //起動メッセージ
  USBSerial.printf("Join StampFly!\r\n");
}

void receive(void)
{

}

void get_gyro(void)
{

}

void pid_control()
{

}

void mixing()
{

}

void filter()
{

}

void set_duty(float fr, float rr, float rl, float fl)
{

}


void arm(void)
{
  receive();
  get_gyro();
  pid_control();
  mixing();
  filter();
  set_duty();
}

void disarm(void)
{
  stop_motor();
  reset_pid();
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
  //以下に自分のコードを記述してください
  
  if(Mode = 0)
  {
    disarm();
  }
  else{
    arm();
  }


  imu_update();
  FastLED.show();
  //End of Loop_400Hz function
}
