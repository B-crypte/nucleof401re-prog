/*エンコーダによる制御処理*/
#include "mbed.h"
#include "encoder_control.h"

InterruptIn Ena(D9,PullUp);   //quadrature A phase
InterruptIn Enb(D8,PullUp);   //quadrature B phase

//エンコーダカウンタ
volatile static int enc_count;   //現在の値
volatile static int old_count;   //enc_count更新前の値を保持

//カウンタ初期化
int init_enc(void)
{
    enc_count = 0;
    old_count = 0;
    
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
    }else{          //逆転
        enc_count--;      //カウントDec
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
    }else{          //正転
        enc_count++;      //カウントInc
    }
    if(enc_count == ENCODER_CPR) { //カウント範囲の補正
        //enc_count = 0;
    }
}
//カウント値取得
int getcnt_enc(int *pi_now,int *pi_old)
{
    *pi_now = enc_count;
    *pi_old = old_count;
    return 0;
}
//カウント記録
int rem_enc(void)
{
    old_count = enc_count;
    return 0;
}
//エンコーダカウンタ任意値設定
int setcnt_enc(int set)
{
    enc_count = set;
    return 0;
}
//カウント値読み込み
int readcnt_enc(void)
{
    return enc_count;
}