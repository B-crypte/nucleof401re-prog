#pragma once

#define PI 3.1415        //円周率
#define ENCODER_CPR 1024 // CPR(Count Per Revolution
#define PI_2 2.0*PI

//プロトタイプ宣言
//カウンタ初期化
int init_enc(void);
//カウントアップ
void cntUP_enc(void);
//カウント値取得
int getcnt_enc(int *pi_now, int *pi_old);
//カウント記録
int rem_enc(void);
//エンコーダカウンタ任意値設定
int setcnt_enc(int set);
//カウント値読み込み
int readcnt_enc(void);
void encoder_a_cnter(void);
void encoder_b_cnter(void);
int read_rot_dir(void);
//エンコーダカウントのリセット
void reset_enc_cnt();
//回転速度の測定
float mesu_rot_speed();
//現在の回転速度を取得
float read_rot_spd();
// 100回平均した回転周期の値読み取り関数
float read_rot_interval(void);
void mesu_rot_interval(void);