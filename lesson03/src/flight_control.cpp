/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

//
// StampFly Flight Control Main Module
//
// Desigend by Kouhei Ito 2023~2024
//
// 2024-06-20 高度制御改良　段差対応
// 2024-06-25 高度制御改良　上昇持続バグ修正
// 2024-06-29 自動離陸追加
// 2024-06-29 自動着陸追加
// 2024-06-29 送信機OFFで自動着陸
// 2024-06-29 着陸時、Madgwick Filter Off
// 2024-07-21 flip関数追加、高度センサの測定限界で自動降下（暫定版）
// 2024-08-10 Acroモードで高度制御働かないバグを修正

#include "flight_control.hpp"
#include "rc.hpp"
#include "pid.hpp"
#include "sensor.hpp"
#include "led.hpp"
#include "telemetry.hpp"
#include "button.hpp"
#include "buzzer.h"

// モータPWM出力Pinのアサイン
// Motor PWM Pin
const int pwmFrontLeft  = 5;
const int pwmFrontRight = 42;
const int pwmRearLeft   = 10;
const int pwmRearRight  = 41;

// モータPWM周波数
// Motor PWM Frequency
const int freq = 150000;

// PWM分解能
// PWM Resolution
const int resolution = 8;

// モータチャンネルのアサイン
// Motor Channel
const int FrontLeft_motor  = 0;
const int FrontRight_motor = 1;
const int RearLeft_motor   = 2;
const int RearRight_motor  = 3;

// 制御周期
// Control period
float Control_period = 0.0025f;  // 400Hz

// PID Gain
// Rate control PID gain
const float Roll_rate_kp  = 0.65f;
const float Roll_rate_ti  = 0.7f;
const float Roll_rate_td  = 0.01;
const float Roll_rate_eta = 0.125f;

const float Pitch_rate_kp  = 0.95f;
const float Pitch_rate_ti  = 0.7f;
const float Pitch_rate_td  = 0.025f;
const float Pitch_rate_eta = 0.125f;

const float Yaw_rate_kp  = 3.0f;
const float Yaw_rate_ti  = 0.8f;
const float Yaw_rate_td  = 0.01f;
const float Yaw_rate_eta = 0.125f;

// Angle control PID gain
const float Rall_angle_kp  = 5.0f;  // 8.0
const float Rall_angle_ti  = 4.0f;
const float Rall_angle_td  = 0.04f;
const float Rall_angle_eta = 0.125f;

const float Pitch_angle_kp  = 5.0f;  // 8.0
const float Pitch_angle_ti  = 4.0f;
const float Pitch_angle_td  = 0.04f;
const float Pitch_angle_eta = 0.125f;

// Altitude control PID gain
const float alt_kp     = 0.38f;  // 5.0//soso 0.5
const float alt_ti     = 10.0f;  // 200.0//soso 10.0
const float alt_td     = 0.5f;   // 0.5//soso 0.5
const float alt_eta    = 0.125f;
const float alt_period = 0.0333;

const float z_dot_kp  = 0.08f;  // 0.35//soso 0.1
const float z_dot_ti  = 0.95f;  // 500.0//soso 0.95
const float z_dot_td  = 0.08f;  // 0.15//1.0//soso 0.08
const float z_dot_eta = 0.125f;

const float Duty_bias_up   = 1.581f;  // Altitude Control parameter　Itolab 1.589 M5Stack 1.581
const float Duty_bias_down = 1.578f;  // Auto landing  parameter Itolab 1.578 M5Stack 1.578

// Times
volatile float Elapsed_time     = 0.0f;//初期化が終了してからの経過時間
volatile float Old_Elapsed_time = 0.0f;//インターバルを計算するための前の時間保持
volatile float Interval_time    = 0.0f;//インターバル（制御周期）
volatile uint32_t S_time = 0, E_time = 0, D_time = 0;//, Dt_time = 0;

// Counter
//uint8_t AngleControlCounter   = 0;
//uint16_t RateControlCounter   = 0;
uint16_t OffsetCounter        = 0;//ジャイロオフセットをデータの平均から求めるためのカウンタ
uint16_t Auto_takeoff_counter = 0;//自動離陸のシーケンスを流すためのカウンタ

// Motor Duty
volatile float FrontRight_motor_duty = 0.0f;
volatile float FrontLeft_motor_duty  = 0.0f;
volatile float RearRight_motor_duty  = 0.0f;
volatile float RearLeft_motor_duty   = 0.0f;

// 制御目標
// PID Control reference
// 角速度目標値
// Rate reference
volatile float Roll_rate_reference = 0.0f, Pitch_rate_reference = 0.0f, Yaw_rate_reference = 0.0f;
// 角度目標値
// Angle reference
volatile float Roll_angle_reference = 0.0f, Pitch_angle_reference = 0.0f, Yaw_angle_reference = 0.0f;
// 舵角指令値
// Commanad
// スロットル指令値
// Throttle
volatile float Thrust_command = 0.0f, Thrust_command2 = 0.0f;
// 角速度指令値
// Rate command
volatile float Roll_rate_command = 0.0f, Pitch_rate_command = 0.0f, Yaw_rate_command = 0.0f;
// 角度指令値
// Angle comannd
volatile float Roll_angle_command = 0.0f, Pitch_angle_command = 0.0f, Yaw_angle_command = 0.0f;

// Offset
volatile float Roll_angle_offset = 0.0f, Pitch_angle_offset = 0.0f, Yaw_angle_offset = 0.0f;
volatile float Elevator_center = 0.0f, Aileron_center = 0.0f, Rudder_center = 0.0f;

// Machine state & flag
//float Timevalue          = 0.0f;
volatile uint8_t Mode    = INIT_MODE;
volatile uint8_t OldMode = INIT_MODE;
uint8_t Control_mode     = ANGLECONTROL;
// volatile uint8_t LockMode=0;
float Motor_on_duty_threshold         = 0.1f;
float Angle_control_on_duty_threshold = 0.5f;
//int8_t BtnA_counter                   = 0;
//uint8_t BtnA_on_flag                  = 0;
//uint8_t BtnA_off_flag                 = 1;
volatile uint8_t Loop_flag            = 0;
// volatile uint8_t Angle_control_flag = 0;
//uint8_t Stick_return_flag     = 0;
uint8_t Throttle_control_mode = 0;
uint8_t Landing_state         = 0;
uint8_t OladRange0flag        = 0;//下降してない場合にスラストを減少するための変数

// for flip
//float FliRoll_rate_time          = 2.0;
uint8_t Flip_flag                = 0;
uint16_t Flip_counter            = 0;
float Flip_time                  = 2.0;
volatile uint8_t Ahrs_reset_flag = 0;
float T_flip;

// PID object and etc.
PID p_pid;
PID q_pid;
PID r_pid;
PID phi_pid;
PID theta_pid;
PID psi_pid;
// PID alt;
PID alt_pid;
PID z_dot_pid;
Filter Thrust_filtered;
Filter Duty_fr;
Filter Duty_fl;
Filter Duty_rr;
Filter Duty_rl;

volatile float Thrust0 = 0.0;
uint8_t Alt_flag       = 0;

// 速度目標Z
float Z_dot_ref = 0.0f;

// 高度目標
const float Alt_ref0   = 0.5f;
volatile float Alt_ref = Alt_ref0;

uint8_t ahrs_reset_flag      = 0;
uint8_t last_ahrs_reset_flag = 0;//フラグの変化を見るため

// Function declaration
void init_pwm();
void control_init();
void variable_init(void);
//void get_command(void);
//void angle_control(void);
//void rate_control(void);
//void output_data(void);
//void output_sensor_raw_data(void);
void motor_stop(void);
//uint8_t judge_mode_change(void);
//uint8_t get_arming_button(void);
//uint8_t get_flip_button(void);
//void reset_rate_control(void);
//void reset_angle_control(void);
//uint8_t auto_landing(void);
//float get_trim_duty(float voltage);
//void flip(void);
//float get_rate_ref(float x);//スポーツモードのスティックとレート指令値との対応

// 割り込み関数
// Intrupt function
hw_timer_t* timer = NULL;
void IRAM_ATTR onTimer() {
    Loop_flag = 1;
}

// Initialize Multi copter
void init_copter(void) {
    // Initialize Mode
    Mode = INIT_MODE;

    // Initialaze LED function
    led_init();
    esp_led(0x110000, 1);
    onboard_led1(WHITE, 1);
    onboard_led2(WHITE, 1);
    led_show();
    led_show();
    led_show();

    // Initialize Serial communication
    USBSerial.begin(115200);
    delay(1500);
    USBSerial.printf("Start StampFly!\r\n");

    // Initialize PWM
    init_pwm();
    sensor_init();
    USBSerial.printf("Finish sensor init!\r\n");

    // PID GAIN and etc. Init
    control_init();

    // Initilize Radio control
    rc_init();

    // 割り込み設定
    // Initialize intrupt
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 2500, true);
    timerAlarmEnable(timer);

    // init button G0
    init_button();

    setup_pwm_buzzer();

    USBSerial.printf("Finish StampFly init!\r\n");
    USBSerial.printf("Enjoy Flight!\r\n");
    start_tone();
}

uint32_t Loop_counter = 0;
// Main loop
void loop_400Hz(void) {
    //以下は変更しない
    while(Loop_flag==0);
    Loop_flag = 0;
    //以上は変更しない

    //Start of Loop_400Hz function
    //以下に記述したコードが400Hzで繰り返される
    blink_led();
  
    Loop_counter ++ ;

    //1秒ごとにLoop_counterの値を端末に表示
    if(Loop_counter%400==0) 
        USBSerial.printf("%6d\n\r", Loop_counter);
  
    FastLED.show(64);


    //End of Loop_400Hz function
}

///////////////////////////////////////////////////////////////////
//  PID control gain setting
//
//  Sets the gain of PID control.
//
//  Function usage
//  PID.set_parameter(PGAIN, IGAIN, DGAIN, TC, STEP)
//
//  PGAIN: PID Proportional Gain
//  IGAIN: PID Integral Gain
//   *The larger the value of integral gain, the smaller the effect of integral control.
//  DGAIN: PID Differential Gain
//  TC:    Time constant for Differential control filter
//  STEP:  Control period
//
//  Example
//  Set roll rate control PID gain
//  p_pid.set_parameter(2.5, 10.0, 0.45, 0.01, 0.001);
void control_init(void) {
    // Rate control
    p_pid.set_parameter(Roll_rate_kp, Roll_rate_ti, Roll_rate_td, Roll_rate_eta,
                        Control_period);  // Roll rate control gain
    q_pid.set_parameter(Pitch_rate_kp, Pitch_rate_ti, Pitch_rate_td, Pitch_rate_eta,
                        Control_period);  // Pitch rate control gain
    r_pid.set_parameter(Yaw_rate_kp, Yaw_rate_ti, Yaw_rate_td, Yaw_rate_eta, Control_period);  // Yaw rate control gain

    // Angle control
    phi_pid.set_parameter(Rall_angle_kp, Rall_angle_ti, Rall_angle_td, Rall_angle_eta,
                          Control_period);  // Roll angle control gain
    theta_pid.set_parameter(Pitch_angle_kp, Pitch_angle_ti, Pitch_angle_td, Pitch_angle_eta,
                            Control_period);  // Pitch angle control gain

    // Altitude control
    alt_pid.set_parameter(alt_kp, alt_ti, alt_td, alt_eta, alt_period);
    z_dot_pid.set_parameter(z_dot_kp, z_dot_ti, z_dot_td, alt_eta, alt_period);

    Duty_fl.set_parameter(0.003, Control_period);
    Duty_fr.set_parameter(0.003, Control_period);
    Duty_rl.set_parameter(0.003, Control_period);
    Duty_rr.set_parameter(0.003, Control_period);

    Thrust_filtered.set_parameter(0.01, Control_period);
}
///////////////////////////////////////////////////////////////////

void set_duty_fr(float duty) {
    ledcWrite(FrontRight_motor, (uint32_t)(255 * duty));
}
void set_duty_fl(float duty) {
    ledcWrite(FrontLeft_motor, (uint32_t)(255 * duty));
}
void set_duty_rr(float duty) {
    ledcWrite(RearRight_motor, (uint32_t)(255 * duty));
}
void set_duty_rl(float duty) {
    ledcWrite(RearLeft_motor, (uint32_t)(255 * duty));
}

void init_pwm(void) {
    ledcSetup(FrontLeft_motor, freq, resolution);
    ledcSetup(FrontRight_motor, freq, resolution);
    ledcSetup(RearLeft_motor, freq, resolution);
    ledcSetup(RearRight_motor, freq, resolution);
    ledcAttachPin(pwmFrontLeft, FrontLeft_motor);
    ledcAttachPin(pwmFrontRight, FrontRight_motor);
    ledcAttachPin(pwmRearLeft, RearLeft_motor);
    ledcAttachPin(pwmRearRight, RearRight_motor);
}

uint8_t get_arming_button(void) {
    static int8_t chatta = 0;
    static uint8_t state = 0;
    if ((int)Stick[BUTTON_ARM] == 1) {
        chatta++;
        if (chatta > 10) {
            chatta = 10;
            state  = 1;
        }
    } else {
        chatta--;
        if (chatta < -10) {
            chatta = -10;
            state  = 0;
        }
    }
    return state;
}

uint8_t get_flip_button(void) {
    static int8_t chatta = 0;
    uint8_t state;

    state = 0;
    if ((int)Stick[BUTTON_FLIP] == 1) {
        chatta++;
        if (chatta > 10) {
            chatta = 0;
            state  = 1;
        }
    } else {
        chatta = 0;
        state  = 0;
    }
    return state;
}

void motor_stop(void) {
    set_duty_fr(0.0);
    set_duty_fl(0.0);
    set_duty_rr(0.0);
    set_duty_rl(0.0);
}