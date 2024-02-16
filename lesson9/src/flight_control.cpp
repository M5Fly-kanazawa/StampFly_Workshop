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
float Ref_t, Ref_p, Ref_q, Ref_r; 
float RateP, RateQ, RateR;
float DeltaP, DeltaQ, DeltaR;
float DutyFR, DutyRR, DutyRL, DutyFL;
float filteredFR, filteredRR, filteredRL, filteredFL;

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

  //PIDリセット
  reset_pid();

  //割り込み設定
  init_interrupt();

  //起動メッセージ
  USBSerial.printf("Join StampFly!\r\n");
}

void receive(void)
{
  Ref_t = Stick[THROTTLE];
  Ref_p = 60*PI*Stick[AILERON]/180.0; 
  Ref_q = 60*PI*Stick[ELEVATOR]/180.0; 
  Ref_r = 60*PI*Stick[RUDDER]/180.0;
}

void get_gyro(void)
{
  RateP = imu_get_gyro_x();
  RateQ = imu_get_gyro_y();
  RateR = imu_get_gyro_z();
}

void pid_control(void)
{
  DeltaP = pid_P(Ref_p, RateP);
  DeltaQ = pid_Q(Ref_q, RateQ);
  DeltaR = pid_R(Ref_r, RateR);
}

void mixing(void)
{
  DutyFR = Ref_t -10.87*DeltaP + 10.87*DeltaQ + 0.25*DeltaR;
  DutyRR = Ref_t -10.87*DeltaP - 10.87*DeltaQ - 0.25*DeltaR;
  DutyRL = Ref_t +10.87*DeltaP - 10.87*DeltaQ + 0.25*DeltaR;
  DutyFL = Ref_t +10.87*DeltaP + 10.87*DeltaQ - 0.25*DeltaR;
}

void filter(void)
{
  float kf = 0.8;
  filteredFR = (1 - kf) * filteredFR + kf * DutyFR;
  filteredRR = (1 - kf) * filteredFR + kf * DutyFR;
  filteredRL = (1 - kf) * filteredFR + kf * DutyFR;
  filteredFL = (1 - kf) * filteredFR + kf * DutyFR;
}

void set_duty(void)
{
  set_motor_duty(FRONT_RIGHT_MOTOR, filteredFR);
  set_motor_duty(REAR_RIGHT_MOTOR,  filteredRR);
  set_motor_duty(REAR_LEFT_MOTOR,   filteredRL);
  set_motor_duty(FRONT_LEFT_MOTOR,  filteredFL);
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
  
  if(Mode == 0)
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
