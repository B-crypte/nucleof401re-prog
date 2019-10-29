#include "mbed.h"
#include "motor_control.h"
#include "encoder_control.h"

PwmOut Mt1(D11);     //motor Input1
PwmOut Mt2(D10);     //motor Input2
Ticker  Update_mtr;  //

static float en1 = 0.0f;
static float en2 = 0.0f;
static float MVn = 0.0f;
static float pre_omega = 0.0f;
static float ratio = 0.0f;
static float mtr_spd;    //[rad/s]

//初期化
void init_mtr(void){
    en1 = 0.0f;
    en2 = 0.0f;
    MVn = 0.0f;
    pre_omega = 0.0f;
    ratio = 0.0f;
    Mt1.write(1.0f);
    Mt2.write(1.0f);
    Mt1.period_ms(1);
    Mt2.period_ms(1);
    set_goal_rpm(240.0f);
}
//PWM設定
void write_pwm_mtr(int no,float per){
    switch(no) {
        case 0:
            Mt1.write(per);
            break;
        case 1:
            Mt2.write(per);
            break;
        default:
        {}
    }
}
//PID制御用
void pid_ctr_mtr(float per){
    if(per > 1.0f) per = 1.0f;
    if(per < 0.0f) per = 0.0f;
    Mt1.write(1.0f);
    Mt2.write(1.0f-per);
}
//モータ動作状態読み取り
float readstate_mtr(int no_mtr){
    switch(no_mtr) {
        case 0:
            return Mt1.read();
        case 1:
            return Mt2.read();
        default:
        {}
    }
    return 0;
}
void start_mtr(){
    //第二引数，時間指定を変数，マクロですると動作がおかしくなる
    Update_mtr.attach(&motor_pid,0.01f);
}
void stop_mtr(){
    write_pwm_mtr(1,1.0f);
    Update_mtr.detach();
}
void change_spd(float set_spd){
    mtr_spd = set_spd;
}
//RPMから角周波数[rad/s]を求め，目標値にセットする
void set_goal_rpm(float s_rpm){
    mtr_spd = 2.0f * PI * (s_rpm/60.0f);
}
void ctr_spd(float add){
    mtr_spd += add;
    if(mtr_spd < 0.0f)
        mtr_spd = 0.0f;
}
//モータの角速度制御
void motor_pid(void){
    float en,ep,ei,ed;      //各偏差
    float rate_reduction = 114.7f;             //減速比
    float rate_pwm = 15000.f*2.0f*PI/60.0f;    //PWMレートが最高速度で1.0になるようにする

    //角速度の計算[rad/s]
    //getcnt_enc(&pre_cnt,&old_cnt);
    //pre_omega = pre_cnt*(((2.0f*PI)/1024.0f)/0.01f);
    //関数呼び出し，角速度の計算[rad/s]
    pre_omega = mesu_rot_speed()*rate_reduction;
    reset_enc_cnt();
    //偏差=目標値-現在の角速度
    en = (mtr_spd) - pre_omega;  
    //en = 20.0 - pre_omega;
    //P制御による制御量を計算
    //MVn += Kp*(en - en1);
    ep = en - en1;
    ei = dT * (en+en1)/2.0f;
    //ed = (en-2*en1+en2)/dT;
    //MVn += Kp * (ep+(ei/Ti)+Td*ed);
    MVn = Kp*(ep + ei/Ti);
    //PWMの比率で結果を反映(最大角速度：-[rad/s])
    //mt=15000rpm,減速比114.7
    ratio += MVn/rate_pwm;
    pid_ctr_mtr(ratio);
    //偏差を記録
    en2 = en1;
    en1 = en;
}
//デューティ比率（操作値）
float read_ratio(){
    return ratio;
}