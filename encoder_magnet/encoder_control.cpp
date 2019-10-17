/*エンコーダによる制御処理*/
#include "mbed.h"
#include "encoder_control.h"

InterruptIn Ena(D9,PullUp);   //quadrature A phase
InterruptIn Enb(D8,PullUp);   //quadrature B phase

InterruptIn Index_w(D7,PullUp); //m1,m2:absolute zero pos.,m3:W sign
Ticker Mesu_encoder;
Timer rot_t1;

//MDxの定義で変更
//InterruptIn LSB(PA_8,PullUp); //step/dir mode Least Sign Bit
//DigitalIn Dir(PA_9);          //direction of rotation
//DigitalIn Mt_U(PA_8,PullUp);  //U sign(pahse1)
//DigitalIn Mt_V(PA_9,PullUp);  //V sign(phase2)
DigitalIn DO(D4);               //Data Output Serial interface
DigitalIn PWM_LSB(D2);          //PWM LSB in mode3
DigitalOut CS(D6);              //chip select
DigitalOut Prog(PB_15);         //otp program(mode set)
DigitalOut CLK(D5);             //clock(trigger input)

//エンコーダカウンタ(のちに構造化することも考慮)
volatile static int enc_count;  //現在の値
volatile static int old_count;  //enc_count更新前の値を保持
volatile static int timecnt;    //1回転した回数をカウント
static float ave_interval;      //1回転あたりの平均時間（回転周期）
static int rot_dir;             //回転方向
static float pre_spd;

//カウンタ初期化
int init_enc(void){
    enc_count = 0;
    old_count = 0;
    rot_dir = 0;
    ave_interval = 0.0;

    Ena.rise(&encoder_a_cnter);   //A相立ち上がり
    Enb.rise(&encoder_b_cnter);   //B相立ち上がり
    Ena.fall(&encoder_a_cnter);   //A相立ち下がり
    Enb.fall(&encoder_b_cnter);   //B相立ち下がり
    Index_w.rise(&mesu_rot_interval);   //Z相立ち上がり
    return 0;
}
//A相　割り込み
void encoder_a_cnter(void){
    //判別変数
    short rot = Ena^Enb;      //回転方向

    if(rot == 1){   //正転
        enc_count++;      //カウントInc
        rot_dir = 0;
    }else{          //逆転
        enc_count--;      //カウントDec
        rot_dir = 1;
    }
    if(enc_count == ENCODER_CPR) { //カウント範囲の補正
        //enc_count = 0;
    }
}
//B相　割り込み
void encoder_b_cnter(void){
    short rot = Ena^Enb;  //回転方向

    if(rot == 1){   //逆転
        enc_count--;      //カウントDec
        rot_dir = 1;
    }else{          //正転
        enc_count++;      //カウントInc
        rot_dir = 0;
    }
    if(enc_count == ENCODER_CPR) { //カウント範囲の補正
        //enc_count = 0;
    }
}
//カウント値取得
int getcnt_enc(int *pi_now,int *pi_old){
    *pi_now = enc_count;
    *pi_old = old_count;
    return 0;
}
//カウント記録
int rem_enc(void){
    old_count = enc_count;
    return 0;
}
//エンコーダカウンタ任意値設定
int setcnt_enc(int set){
    enc_count = set;
    return 0;
}
//カウント値読み込み
int readcnt_enc(void){
    return enc_count;
}
//回転方向の外部参照関数
int read_rot_dir(void){
    return rot_dir;
}
//エンコーダカウントのリセット
void reset_enc_cnt(){
    enc_count = 0;
    old_count = 0;
}
//回転速度の測定
float mesu_rot_speed(){
    int pre_cnt = enc_count - old_count;
    
    //current_spd[rad/s]
    pre_spd  = pre_cnt*(((2.0*PI)/1024.0)/0.01);
    //現在のカウント値を記録
    old_count = enc_count;

    return pre_spd;
}
//現在の回転速度を取得
float read_rot_spd(){
    //回転速度[rpm]
    float rot_rpm = pre_spd * 60.0 / (2.0*PI); 
    if(rot_rpm<0){
        return  rot_rpm*(-1);
    }else{
        return  rot_rpm;
    }
}
//indexw端子を使った回転周期の測定
void mesu_rot_interval(void){
    timecnt++;
    if(timecnt==10){
        rot_t1.stop();
        timecnt=0;
        ave_interval = rot_t1.read_ms()/10.0;
        rot_t1.reset();
        rot_t1.start();
    }
}
//10回平均した回転周期の値読み取り関数
float read_rot_interval(void){
    return ave_interval;
}
