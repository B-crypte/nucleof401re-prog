#pragma once

#define ENCODER_CPR 1920    //CPR(Count Per Revolution

//プロトタイプ宣言
//カウンタ初期化
int init_enc(void);
//カウントアップ
void cntUP_enc(void);
//カウント値取得
int getcnt_enc(int *pi_now,int *pi_old);
//カウント記録
int rem_enc(void);
//エンコーダカウンタ任意値設定
int setcnt_enc(int set);
//カウント値読み込み
int readcnt_enc(void);

void encoder_a_cnter(void);
void encoder_b_cnter(void);