#pragma once

//pidパラメータ
#define Kp 0.8f     //比例
#define Ki 0.5f
#define Kd 0.01f
#define Ti 0.06f    //微分
#define Td 0.0f     //積分
#define dT 0.01f    //PID制御　タイマ割り込み時間

//PIDによるモータ制御の設定（未完成）
void motor_pid(void);
//初期化
void init_mtr(void);
//PWM設定
void write_pwm_mtr(int no,float per);
//PID制御用
void pid_ctr_mtr(float per);
//モータ動作状態読み取り
float readstate_mtr(int no_mtr);
//モータ動作開始
void start_mtr();
//モータ動作停止
void stop_mtr();
//角速度を取得
float get_speed();
void change_spd(float set_spd);
void ctr_spd(float add);
float read_ratio();
//RPMから角周波数[rad/s]を求め，目標値にセットする
void set_goal_rpm(float s_rpm);
//第一引数が範囲内を超えないように調整する関数
float limit(float target,float min,float max);
float pid(float xin, float kp, float ki, float kd, float* work,float dt);
void spd_ctr_pid();
void pid_value_disp();