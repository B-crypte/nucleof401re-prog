#pragma once

//pidパラメータ
#define Kp (double)1.0
#define Ti (double)0.06
#define Td (double)0.012
#define dT 0.01   //PID制御　タイマ割り込み時間

//PIDによるモータ制御の設定（未完成）
void motor_pid(void);
//初期化
void init_mtr(void);
//PWM設定
void write_pwm_mtr(int no,double per);
//PID制御用
void pid_ctr_mtr(double per);
//モータ動作状態読み取り
float readstate_mtr(int no_mtr);
//モータ動作開始
void start_mtr();
//モータ動作停止
void stop_mtr();
//角速度を取得
double get_speed();
void change_spd(double set_spd);
void ctr_spd(double add);
