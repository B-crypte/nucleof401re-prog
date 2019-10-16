#include "mbed.h"
#include "encoder_control.h"
#include "motor_control.h"

#define CCW 0               //clockwise operation (1bit;0:cw,1:ccw)
#define ZEROPOS 0x00        //programmed zero position (10bit)
#define INDEX 0             //index bit width (1bit)
#define DIVX 0x0            //incremental resolution (2bit;mode_10bit,mode_7bit,mode_5bit)
#define MDX  0x0            //incremental mode (2bit;m1:quadrature,m2:step/dir,m3:motor)

DigitalIn MagINC(PA_12);    //magnitude increase
DigitalIn MagDEC(PA_11);    //magnitude decrease
InterruptIn b1(USER_BUTTON);

//global 
int cnt;      //count
bool dir;
unsigned char current;   //記憶値

//test moter
void motor_tgl(void){
    if(readstate_mtr(0)==1.0){
        rot_speed_chk_start();
        write_pwm_mtr(0,0.95);   //低速回転
    }else{
        write_pwm_mtr(0,1.0);   //停止
    }
}

//main func.
int main(){
    int buf;
    float rpm;
    //初期化シーケンス
    init_enc(); //エンコーダIC
    init_mtr(); //モータドライバIC
    //モータ動作確認
    b1.rise(&motor_tgl);
    //roop
    while (1) {
        getcnt_enc(&cnt,&buf);
        dir = get_rot_dir();
        rpm = get_rot_spd();
        printf("MagInc:%d,MagDec:%d\n",(bool)MagINC,(bool)MagDEC);
        printf("rpm:%.1f,dir:%2d\n",rpm,dir); 
        printf("cnt:%4d\n\e[3A",cnt);
    }
}