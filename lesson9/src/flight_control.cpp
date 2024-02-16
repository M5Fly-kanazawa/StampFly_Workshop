//Lesson 2
//スロットルレーバーでモータの回転が変わるようにする
#include "flight_control.hpp"
#include "motor.hpp"
#include "rc.hpp"
#include "led.hpp"
#include "imu.hpp"
#include <math.h>
#include "pid.hpp"
#include "battery.hpp"

//Global variable
const float Control_period = 0.0025f;//400Hz //制御周期
volatile uint8_t Loop_flag = 0;
uint32_t Loop_counter = 0;

//ここから新しくかいたよ
uint8_t Mode = 2;
float Ref_t, Ref_p, Ref_q, Ref_r; 
float RateP, RateQ, RateR;
float DeltaP, DeltaQ, DeltaR;
float DutyFR, DutyRR, DutyRL, DutyFL;
float filteredFR, filteredRR, filteredRL, filteredFL;
float BiasP=0.0, BiasQ=0.0, BiasR=0.0;

void get_bias(void);

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
  board_tail_led(0,0);
  board_bottom_led(0,0);
  stamp_led(0,0);

  //IMUの設定
  imu_init();
  imu_update();

  //Battery
  init_battery();

  //PIDリセット
  reset_pid();

  //割り込み設定
  init_interrupt();

  //起動メッセージ
  USBSerial.printf("Join StampFly!\r\n");
  
}

void get_bias(void)
{
  if(Loop_counter<1600)
  { 
    BiasP = BiasP + imu_get_gyro_x();
    BiasQ = BiasQ + imu_get_gyro_y();
    BiasR = BiasR + imu_get_gyro_z();
  }
  else
  {
    BiasP = BiasP/1600.0;
    BiasQ = BiasQ/1600.0;
    BiasR = BiasR/1600.0;
    Mode = 0;
  }
  Loop_counter ++;
  USBSerial.printf("%6.9f\n", BiasP);
  board_tail_led(PERPLE, 1);
}

void receive(void)
{
  Ref_t = Stick[THROTTLE];
  Ref_p = 120*PI*Stick[AILERON]/180.0; 
  Ref_q = 120*PI*Stick[ELEVATOR]/180.0; 
  Ref_r = 120*PI*Stick[RUDDER]/180.0;
}

void get_gyro(void)
{
  RateP = imu_get_gyro_x() - BiasP;
  RateQ = imu_get_gyro_y() - BiasQ;
  RateR = imu_get_gyro_z() - BiasR;
}

void pid_control(void)
{
  DeltaP = pid_P(Ref_p, RateP, 1);
  DeltaQ = pid_Q(Ref_q, RateQ, 1);
  DeltaR = pid_R(Ref_r, RateR, 1);
}

void mixing(void)
{
  if (Ref_t <0.1)
  {
    stop_motor();
    reset_pid();
    DutyFR = 0.0;
    DutyRR = 0.0;
    DutyRL = 0.0;
    DutyFL = 0.0;
    filteredFR = 0.0;
    filteredRR = 0.0;
    filteredRL = 0.0;
    filteredFL = 0.0;
  }
  else
  {
    DutyFR = Ref_t +(-0.25*DeltaP + 0.25*DeltaQ + 0.25*DeltaR)/3.7;
    DutyRR = Ref_t +(-0.25*DeltaP - 0.25*DeltaQ - 0.25*DeltaR)/3.7;
    DutyRL = Ref_t +( 0.25*DeltaP - 0.25*DeltaQ + 0.25*DeltaR)/3.7;
    DutyFL = Ref_t +( 0.25*DeltaP + 0.25*DeltaQ - 0.25*DeltaR)/3.7;
  }

}

void filter(void)
{
  float kf = 0.8;
  filteredFR = (1 - kf) * filteredFR + kf * DutyFR;
  filteredRR = (1 - kf) * filteredRR + kf * DutyRR;
  filteredRL = (1 - kf) * filteredRL + kf * DutyRL;
  filteredFL = (1 - kf) * filteredFL + kf * DutyFL;
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
  board_tail_led(YELLOW, 1);

  #if 0
  USBSerial.printf(">BiasP:%9.6f\n", BiasP);
  USBSerial.printf(">RateP:%9.6f\n", RateP);
  USBSerial.printf(">BiasQ:%9.6f\n", BiasQ);
  USBSerial.printf(">RateQ:%9.6f\n", RateQ);
  USBSerial.printf(">BiasR:%9.6f\n", BiasR);
  USBSerial.printf(">RateR:%9.6f\n", RateR);
  USBSerial.printf(">filteredFR:%9.6f\n", filteredFR);
  USBSerial.printf(">filteredRR:%9.6f\n", filteredRR);
  USBSerial.printf(">filteredRL:%9.6f\n", filteredRL);
  USBSerial.printf(">filteredFL:%9.6f\n", filteredFL);
  USBSerial.printf(">bat:%9.6f\n", get_voltage());
  #endif
}

void disarm(void)
{
  stop_motor();
  reset_pid();
  board_tail_led(GREEN, 1);
}

void mode(void)
{
  static uint16_t flag = 0;
  static uint16_t state =0;
  
  if(Mode == 2)return;

  if (Stick[BUTTON_ARM] > 0)flag++;
  else flag = 0;

  if (flag == 30){
      Mode = (~Mode)&1;
      state = 1;
  }

  if(state == 0 )flag ++;
  else if(!(Stick[BUTTON_ARM] > 0.0))
  {
    flag = 0;
    state = 0;
  }
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
  
  mode();
  if (Mode == 2)
  {
    get_bias();
  }
  else if(Mode == 0)
  {
    disarm();
  }
  else{
    arm();
  }

  if(get_voltage()<3.3)board_tail_led(POWEROFFCOLOR, 1);
  USBSerial.printf(">bat:%9.6f\n", get_voltage());

  imu_update();
  FastLED.show();
  //End of Loop_400Hz function
}
