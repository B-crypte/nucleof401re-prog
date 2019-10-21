#include "mbed.h"
#include "motor_control.h"
#include "encoder_control.h"

PwmOut Mt1(D11);     //motor Input1
PwmOut Mt2(D10);     //motor Input2
Ticker  Update_mtr;  //

static float en1 = 0;
static float en2 = 0;
static float MVn = 0;
static float pre_omega = 0.0;
static float ratio = 0;
static float mtr_spd = 3.12*2*10; //[rad/s]

//初期化
void init_mtr(void)
{
    en1 = 0;
    en2 = 0;
    MVn = 0;
    pre_omega = 0.0;
    ratio = 0;
    Mt1.write(1);
    Mt2.write(1);
    Mt1.period_ms(1);
    Mt2.period_ms(1);
}
//PWM設定
void write_pwm_mtr(int no,float per)
{
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
void pid_ctr_mtr(float per)
{
    if(per > 1.0f) per = 1.0f;
    if(per < 0.0f) per = 0.0f;
    Mt1.write(1.0f);
    Mt2.write(1.0f-per);
}
//モータ動作状態読み取り
float readstate_mtr(int no_mtr)
{
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
void start_mtr()
{
    //第二引数，時間指定を変数，マクロですると動作がおかしくなる
    Update_mtr.attach(&motor_pid,0.01);
}
void stop_mtr()
{
    write_pwm_mtr(1,1.0);
    Update_mtr.detach();
}
void change_spd(float set_spd)
{
    mtr_spd = set_spd;
}
void ctr_spd(float add)
{
    mtr_spd += add;
    if(mtr_spd < 0.0f)
        mtr_spd = 0.0f;
}
//モータの角速度制御
void motor_pid(void)
{
    int pre_cnt,old_cnt;
    float en,ep,ei,ed;

    //getcnt_enc(&pre_cnt,&old_cnt);
    //角速度の計算[rad/s]
    //pre_omega = pre_cnt*(((2.0f*PI)/1024.0f)/0.01f);
    pre_omega = mesu_rot_speed();
    reset_enc_cnt();
    //偏差:目標値3.14[rad/s]
    en = (mtr_spd) - pre_omega;  
    //en = 20.0 - pre_omega;
    //P制御による制御量を計算
    //MVn += Kp*(en - en1);
    ep = en - en1;
    ei = dT * (en+en1)/2.0f;
    //ed = (en-2*en1+en2)/dT;
    //MVn += Kp * (ep+(ei/Ti)+Td*ed);
    MVn = Kp*(ep + ei/Ti);
    //PWMの比率で結果を反映(最大角速度：123.684[rad/s])
    //mt=15000rpm,減速比12.7,シャフト(mt/12.7)=1181.10[rpm]
    //1181.10*(60/2π)=11278.696[rad/s]
    ratio += MVn/1000.0f;
    pid_ctr_mtr(ratio);
    //偏差を記録
    en2 = en1;
    en1 = en;
}
//デューティ比率（操作値）
float read_ratio(){
    return ratio;
}