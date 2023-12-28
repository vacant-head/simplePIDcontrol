#include <Arduino.h>
#include<math.h>
#include "qei.hpp"

const int control_period = 2000; // us
float constol_freq = 1.0f / (float)control_period;
int interval;
int preinterval = 0;
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
double duty = 0;

double deg_ref;
double vel_ref;
double err;
double gain_p_deg;
double gain_i_deg;
double gain_d_deg;

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
  motorSetup(motor1, 2000);
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

  pot_p = analogRead(pot_pin_p) * pot_calc;
  pot_i = analogRead(pot_pin_i) * pot_calc;
  pot_d = analogRead(pot_pin_d) * pot_calc;

  // Serial.printf("%lf,%lf,%lf\n\r",pot_p,pot_i,pot_d);

  gain_p_deg = 1.0 * (0.01 * pot_p + 0.02);
  gain_i_deg = 1.0 * (10 * pot_i + 4.196);
  gain_d_deg = 1.0 * (0.000001 * pot_d + 0.000001);

  Serial.printf("%lf_%lf_%f\n", gain_p_deg, gain_i_deg, gain_d_deg*100);

  // PID
  deg_ref = 90.0;                 // 目標値
  err = (deg_ref - nowangle_deg); // 偏差
  Serial.printf("%lf\n\r", err);  // 偏差を出力
  duty = gain_p_deg * err + gain_i_deg * err * control_period * 0.000001 + gain_d_deg * err / (control_period * 0.000001);
  motorDrive(motor1, duty);

  // 制御周期安定化
  interval = micros() - preinterval;
  while (interval < control_period)
  {
    interval = micros() - preinterval;
  }
  preinterval = micros();
}
