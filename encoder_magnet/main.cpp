#include "mbed.h"

#define CCW 0               //clockwise operation (1bit;0:cw,1:ccw)
#define ZEROPOS 0x00        //programmed zero position (10bit)
#define INDEX 0             //index bit width (1bit)
#define DIVX 0x0            //incremental resolution (2bit;mode_10bit,mode_7bit,mode_5bit)
#define MDX  0x0            //incremental mode (2bit;m1:quadrature,m2:step/dir,m3:motor)

DigitalOut CS(PB_10);       //chip select
DigitalOut Prog(PB_15);     //otp program(mode set)
DigitalOut CLK(PB_4);       //clock(trigger input)
DigitalIn MagINC(PA_12);    //magnitude increase
DigitalIn MagDEC(PA_11);    //magnitude decrease
//MDxの定義で変更
InterruptIn Aline(PA_8,PullUp);   //quadrature A phase
InterruptIn Bline(PA_9,PullUp);   //quadrature B phase
//InterruptIn LSB(PA_8,PullUp);   //step/dir mode Least Sign Bit
//DigitalIn Dir(PA_9);            //direction of rotation
//DigitalIn Mt_U(PA_8,PullUp);    //U sign(pahse1)
//DigitalIn Mt_V(PA_9,PullUp);    //V sign(phase2)
DigitalIn DO(PA_10);              //Data Output Serial interface
DigitalIn PWM_LSB(PB_5);          //PWM LSB in mode3

InterruptIn Index_w(PC_7,PullUp); //m1,m2:absolute zero pos.,m3:W sign

Ticker CheckEnc;

//global 
int cnt;      //count
unsigned char dir;
unsigned char current;   //記憶値

//initiarize AD5040 otp program
void init_dev(){
    static short wt_da[16];
    int zdata = ZEROPOS;
    //書き込みデータ準備
    wt_da[0] = CCW;
    wt_da[11]= INDEX;
    wt_da[12] = DIVX&0x01;
    wt_da[13] = (DIVX>>1)&0x01;
    wt_da[14] = MDX&0x01;
    wt_da[15] = (MDX>>1)&0x01;
    for(int i = 0;i < 10;i++){
        wt_da[i+1] = zdata & 0x01;
        zdata >>= 0x01;
    }
    //書き込み開始
    Prog = 1;
    CLK = 0;
    wait_us(1);
    CS = 1;
    wait_us(5);   //書き込み開始待ち時間
    for(int i = 0;i < 16;i++){
        Prog = wt_da[i];
        CLK = 1;wait_us(1);
        CLK = 0;wait_us(1);
    }
    //書き込み終了
    CLK = 0; 
    Prog = 0;  
    CS = 0;
    //Vprog=7.5Vをprogに印加して
    //16パルスCLKを流すと書き込みが適用される
}
//counter reset to 0
void Reset_cnt(){
    cnt = 0;
}
//エンコーダ　回転方向判別関数
unsigned char funcD(unsigned char ft0,unsigned char ft1){
    unsigned char result;

    current = ft1;      //現在値を記録
    ft0 <<=  0x01;      //前の状態値を左シフト
    ft0 &= 0x03;        //下位2bit マスク
    ft1 &= 0x03;        //下位2bit マスク
    result = (ft0 + ft1) & 0x03;    //判別結果をマスク

    return result;      //1：右回転，2：左回転
}
//タイマー割り込みによるエンコーダの検出
void interval_timerw(void){
    unsigned char pd;   //回転判別結果
    unsigned char PD;   //現在値

    PD = Aline + (Bline<<1);
    if(current != PD){
        pd = funcD(current,PD);
        if(pd>=2){
            dir = 1;
            cnt++;
        }else{
            dir = 0;
            cnt--;
        }
    }
}
//
void cul_rpm(void){
    int deff_cnt;
}
//main func.
int main(){
    //interrupt
    Index_w.rise(Reset_cnt);
    //coutinuce timer
    CheckEnc.attach_us(&interval_timerw, 100);
    //main
    while (1) {
        printf("MagInc:%d,MagDec:%d\n",(bool)MagINC,(bool)MagDEC);
        printf("cnt:%6d\ndir:%2d\n\e[3A",cnt,dir);
    }
}