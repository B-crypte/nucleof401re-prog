#include "mbed.h"
#include "motor_control.h"
#include "encoder_control.h"

PwmOut Mt1(D11);     //motor Input1
PwmOut Mt2(D10);     //motor Input2
Ticker  Update_mtr;  //

static double en1 = 0;
static double en2 = 0;
static double MVn = 0;
static double read_omega = 0.0;
static double ratio = 0;
static double mtr_spd = 3.14;

//初期化
void init_mtr(void)
{
    en1 = 0;
    en2 = 0;
    MVn = 0;
    read_omega = 0.0;
    ratio = 0;
    Mt1.write(1);
    Mt2.write(1);
    Mt1.period_ms(1);
    Mt2.period_ms(1);
}
//PWM設定
void write_pwm_mtr(int no,double per)
{
    switch(no) {
        case 0:
            Mt1 = per;
            break;
        case 1:
            Mt2 = per;
            break;
        default:
        {}
    }
}
//PID制御用
void pid_ctr_mtr(double per)
{
    if(per > 1.0) per = 1.0;
    if(per < 0.0) per = 0.0;
    Mt1 = 1;
    Mt2 = 1.0 - per;
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
void change_spd(double set_spd)
{
    mtr_spd = set_spd;
}
void ctr_spd(double add)
{
    mtr_spd += add;
    if(mtr_spd < 0)
        mtr_spd = 0.0;
}
//モータの角速度制御
void motor_pid(void)
{
    int now_cnt,old;
    double en,ep,ei,ed,now_omega;

    //エンコーダカウントの取得
    getcnt_enc(&now_cnt,&old);
    setcnt_enc(0);
    //角速度の計算[rad/s]
    now_omega = now_cnt * (((2.0*PI)/1024.0)/dT);
    read_omega = now_omega;
    //偏差:目標値3.14[rad/s]
    en = (mtr_spd) - now_omega;  
    //en = 20.0 - now_omega;
    //P制御による制御量を計算
    //MVn += Kp*(en - en1);
    ep = en - en1;
    ei = dT * (en+en1)/2;
    //ed = (en-2*en1+en2)/dT;
    //MVn += Kp * (ep+(ei/Ti)+Td*ed);
    MVn = Kp*(ep + ei/Ti);
    //PWMの比率で結果を反映(最大角速度：36.65[rad/s])
    //pid_ctr_mtr(MVn/9.817);
    ratio += MVn/36.65;
    pid_ctr_mtr(ratio);
    //偏差を記録
    en2 = en1;
    en1 = en;
    //printf("%lf\n",now_omega);
}
//角速度を取得
double get_speed()
{
    return read_omega;
}