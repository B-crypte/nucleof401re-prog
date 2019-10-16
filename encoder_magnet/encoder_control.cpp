/*エンコーダによる制御処理*/
#include "mbed.h"
#include "encoder_control.h"

InterruptIn Ena(D9,PullUp);   //quadrature A phase
InterruptIn Enb(D8,PullUp);   //quadrature B phase

InterruptIn Index_w(D7,PullUp); //m1,m2:absolute zero pos.,m3:W sign
Ticker Mesu_encoder;

//MDxの定義で変更
//InterruptIn LSB(PA_8,PullUp); //step/dir mode Least Sign Bit
//DigitalIn Dir(PA_9);          //direction of rotation
//DigitalIn Mt_U(PA_8,PullUp);  //U sign(pahse1)
//DigitalIn Mt_V(PA_9,PullUp);  //V sign(phase2)
DigitalIn DO(D4);               //Data Output Serial interface
DigitalIn PWM_LSB(D2);          //PWM LSB in mode3
DigitalOut CS(D6);              //chip select
DigitalOut Prog(PB_15);         //otp program(mode set)
DigitalOut CLK(D5);          //clock(trigger input)

//エンコーダカウンタ
volatile static int enc_count;   //現在の値
volatile static int old_count;   //enc_count更新前の値を保持
static int dir;
static float rot_rpm;

//カウンタ初期化
int init_enc(void){
    enc_count = 0;
    old_count = 0;
    dir = 0;
    rot_rpm = 0;

    Ena.rise(&encoder_a_cnter);   //A相立ち上がり
    Enb.rise(&encoder_b_cnter);   //B相立ち上がり
    Ena.fall(&encoder_a_cnter);   //A相立ち下がり
    Enb.fall(&encoder_b_cnter);   //B相立ち下がり
    return 0;
}
//カウントアップ
void cntUP_enc(void){
    enc_count++;
    if(enc_count == ENCODER_CPR) {
        //enc_count = 0;
    }
}
//A相　割り込み
void encoder_a_cnter(void){
    //判別変数
    short rot = Ena^Enb;      //回転方向

    if(rot == 1){   //正転
        enc_count++;      //カウントInc
        dir = 0;
    }else{          //逆転
        enc_count--;      //カウントDec
        dir = 1;
    }
    if(enc_count == ENCODER_CPR) { //カウント範囲の補正
        //enc_count = 0;
    }
}
//B相　割り込み
void encoder_b_cnter(void){
    short rot = Ena^Enb;      //回転方向

    if(rot == 1){   //逆転
        enc_count--;      //カウントDec
        dir = 1;
    }else{          //正転
        enc_count++;      //カウントInc
        dir = 0;
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
int get_rot_dir(void){
    return dir;
}
//回転速度の計測開始
void rot_speed_chk_start(){
    enc_count = 0;
    old_count = 0;
    Mesu_encoder.attach(&mesu_rot_speed, 0.01);
}
//回転速度の測定
void mesu_rot_speed(){
    int pre_cnt = enc_count - old_count;
    float pre_spd;
    
    //current_spd[rad/s]
    pre_spd  = pre_cnt*(((2.0*PI)/1024.0)/0.01);
    //RPMに変換[rpm]
    rot_rpm = pre_spd * 60.0 / (2.0*PI);
    //現在のカウント値を記録
    old_count = enc_count;   
}
//現在の回転速度を取得
float get_rot_spd(){
   return  rot_rpm;
}