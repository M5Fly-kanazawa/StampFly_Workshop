//Lesson 2
//スロットルレーバーでモータの回転が変わるようにする
#include "flight_control.hpp"
#include "motor.hpp"
#include "rc.hpp"
#include "led.hpp"

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
  
  //Mixsing Matrix
  const float mix[16]={ 6.20e2,-6.20e2, 6.20e2, 3.94e7,
                        6.20e2,-6.20e2,-6.20e2,-3.94e7,
                        6.20e2, 6.20e2,-6.20e2, 3.94e7,
                        6.20e2, 6.20e2, 6.20e2,-3.94e7};  

  //値調整のスケーリング係数
  float kt = 1/6.20e2;
  float kl = 0.5*1/6.20e2;
  float km = 0.5*1/6.20e2;
  float kn = 0.5*1/3.49e7;
  //操縦信号の取得
  float Thrust0 = 0.6;
  float StkThrust = Stick[THROTTLE];
  float Thrust = kt*(StkThrust - Thrust0);
  float Roll_moment = kl * Stick[AILERON];
  float Pitch_moment = km * Stick[ELEVATOR];
  float Yaw_moment = kn * Stick[RUDDER];
  //Mixing
  float delta_fr = mix[0]*Thrust + mix[1]*Roll_moment + mix[2]*Pitch_moment + mix[3]*Yaw_moment;
  float delta_rr = mix[4]*Thrust + mix[5]*Roll_moment + mix[6]*Pitch_moment + mix[7]*Yaw_moment;
  float delta_rl = mix[8]*Thrust + mix[9]*Roll_moment + mix[10]*Pitch_moment + mix[11]*Yaw_moment;
  float delta_fl = mix[12]*Thrust + mix[13]*Roll_moment + mix[14]*Pitch_moment + mix[15]*Yaw_moment;

  float fr_duty = Thrust0 + delta_fr;
  float rr_duty = Thrust0 + delta_rr;
  float rl_duty = Thrust0 + delta_rl;
  float fl_duty = Thrust0 + delta_fl;
  if (fr_duty > 0.95) fr_duty = 0.95;
  if (fr_duty < 0.0)  fr_duty = 0.0;
  if (rr_duty > 0.95) rr_duty = 0.95;
  if (rr_duty < 0.0)  rr_duty = 0.0;
  if (rl_duty > 0.95) rl_duty = 0.95;
  if (rl_duty < 0.0)  rl_duty = 0.0;
  if (fl_duty > 0.95) fl_duty = 0.95;
  if (fl_duty < 0.0)  fl_duty = 0.0;
  set_motor_duty(FRONT_RIGHT_MOTOR, fr_duty);
  set_motor_duty(REAR_RIGHT_MOTOR,  rr_duty);
  set_motor_duty(REAR_LEFT_MOTOR,  rl_duty);
  set_motor_duty(FRONT_LEFT_MOTOR,  fl_duty);

  board_tail_led(GREEN, 1);
  Loop_counter ++ ;

  //1秒ごとにLoop_counterの値を端末に表示
  //if(Loop_counter%400==0) 
  //  USBSerial.printf("%6d %6.3f\n\r", Loop_counter, Stick[THROTTLE]);
  
  FastLED.show();
  //End of Loop_400Hz function
}
