#include <Arduino.h>
#include<math.h>
#include "qei.hpp"

const int control_period = 2000; // us
float constol_freq = 1.0f / (float)control_period;
int interval;
int preinterval = 0;
int count_period = 0;

const int enc_a = 25;
const int enc_b = 26;
const int enc_z = 27;
const int enc_ppr = 2048;
const double deg_per_pulse = 360.0 / (4.0 * (double)enc_ppr);
int16_t count;
int16_t precount = 0;
int16_t count_def;
double def_deg;
double vel_deg;
double nowangle_deg;

const int pwmResolution = 8;
const int pwmPin1 = 4;
const int dirPin1_1 = 18;
const int dirPin1_2 = 19;
const int channel_motor1 = 1;
int motor1[5] = {pwmPin1, dirPin1_1, dirPin1_2, channel_motor1, -1};
double dutyout = 0;

double deg_ref = 90.0;
double vel_ref;
double err;
double pre_err =0;
double p_calc;
double i_calc = 0;
double d_calc;
double gain_p_deg=0.0913;
double gain_i_deg=0.04;
double gain_d_deg=0.00347;

const int pot_pin_p = 32;
const int pot_pin_i = 33;
const int pot_pin_d = 34;
double pot_p;
double pot_i;
double pot_d;
double pot_calc = 0.000244200; // ポテンショ変換定数1/4095

// モーターのセットアップ
void motorSetup(int motornum[], int freq) // motornum[5] = {pwmPin, dirPin1, dirPin2, channel, default_dir}
{
  pinMode(motornum[0], OUTPUT); // pwm
  pinMode(motornum[1], OUTPUT); // IN1
  pinMode(motornum[2], OUTPUT); // IN2
  ledcSetup(motornum[3], freq, pwmResolution);
  ledcAttachPin(motornum[0], motornum[3]);
  // Serial.printf("setup\n");
}

// モーター駆動用関数
void motorDrive(int Drive_motornum[], double duty_per)
{
  duty_per = duty_per * Drive_motornum[4];
  bool dir = duty_per > 0; // 方向
  double DUTY = pow(2, pwmResolution) * abs(duty_per);
  bool dutycheck = DUTY < 256;
  //Serial.printf("DUTY=%lf", DUTY);
  switch (dutycheck)
  {
  case 1:
    digitalWrite(Drive_motornum[1], dir);  // 方向信号1
    digitalWrite(Drive_motornum[2], !dir); // 方向信号2
    ledcWrite(Drive_motornum[3], DUTY);    // pwm出力
    // Serial.printf("DUTY=%lf", DUTY);
    break;
  case 0:
    digitalWrite(Drive_motornum[1], dir);  // 方向信号1
    digitalWrite(Drive_motornum[2], !dir); // 方向信号2
    ledcWrite(Drive_motornum[3], 255); // duty = 255を出力
    // Serial.printf("DUTYerr=%lf\n", DUTY);
    break;
  }
}

void setup()
{
  Serial.begin(115200);
  qei_setup_x4(PCNT_UNIT_0, enc_a, enc_b);
  motorSetup(motor1, 10000);
  pinMode(pot_pin_p, INPUT);
  pinMode(pot_pin_i, INPUT);
  pinMode(pot_pin_d, INPUT);
}

void loop()
{

  pcnt_get_counter_value(PCNT_UNIT_0, &count);
  // オーバーフロー処理をかく
  count_def = count - precount;
  def_deg = count_def * deg_per_pulse;
  vel_deg = def_deg / (control_period * 0.000001);
  nowangle_deg = count * deg_per_pulse;
  // Serial.printf("%lf", vel_deg));//角速度を表示
  // Serial.printf("%lfdeg\n\r", nowangle_deg);//現在の角度を表示
  precount = count;

  // pot_p = analogRead(pot_pin_p) * pot_calc;
  // pot_i = analogRead(pot_pin_i) * pot_calc;
  // pot_d = analogRead(pot_pin_d) * pot_calc;

  // // Serial.printf("%lf,%lf,%lf\n\r",pot_p,pot_i,pot_d);

  // gain_p_deg = 1.0 * (0.1 * pot_p + 0.0);//0.091
  // gain_i_deg = 1.0 * (0.2 * pot_i + 0.0);//0
  // gain_d_deg = 1.0 * (0.01 * pot_d + 0.0);//0.0003

  // Serial.printf("%f,%f,%f,", gain_p_deg, gain_i_deg, gain_d_deg);

  // PID
  // 目標値生成
  if(count_period <=100)
  {
    deg_ref=90.0;
    //deg_ref =180.0/250*(double)count_period;
  //sin(((double)count_period)/500*M_PI*0.5)*90;
  }
  else
  {
    deg_ref=-90.0;
    //deg_ref = 180.0-180.0/250*((double)count_period-250);
    //sin(((double)count_period)/500*M_PI*0.5)*90;
    if(count_period==200)
    {count_period=0;}
  }

  err = (deg_ref - nowangle_deg); // 偏差
  Serial.printf("%lf,\n\r", err);  // 偏差を出力
  //Serial.printf("<target:%f\n", deg_ref); 
  //Serial.printf("<err:%f\n\r", nowangle_deg);
  //dutyの計算
  p_calc = err;
  i_calc = i_calc+((err+pre_err)*control_period*1.0e-6)/2.0;
  d_calc = (err-pre_err)/(control_period*1.0e-6);
  pre_err = err;
  dutyout = gain_p_deg * p_calc + gain_i_deg * i_calc + gain_d_deg * d_calc;
  motorDrive(motor1, dutyout);
  
  // 制御周期安定化
  interval = micros() - preinterval;
  while (interval < control_period)
  {
    interval = micros() - preinterval;
  }
  preinterval = micros();
  count_period +=1; //制御周期をカウントアップ
}
