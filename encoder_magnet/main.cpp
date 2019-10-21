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
    if(readstate_mtr(1)==1.0){
        start_mtr();
        //write_pwm_mtr(1,0.5);
    }else{
        stop_mtr();   //停止
        //write_pwm_mtr(1,1.0);
    }
}

//main func.
int main(){
    int buf,i;
    float rpm_ave;
    float rpm_total;
    float interval_ave;
    float prirpm;
    float rto;

    //初期化シーケンス
    init_enc(); //エンコーダIC
    init_mtr(); //モータドライバIC
    //モータ制御確認
    b1.rise(&motor_tgl);
    //roop
    i = 0;
    while (1) {
        getcnt_enc(&cnt,&buf);
        dir = read_rot_dir();
        rto = read_ratio();
        interval_ave = read_rot_interval();
        if(i < 10){
            rpm_total += read_rot_spd();
            i++;
        }else{
            rpm_ave = rpm_total / 10.0f;
            rpm_total = 0.0f;
            i = 0;
        }
        printf("MagInc:%d,MagDec:%d\n",(bool)MagINC,(bool)MagDEC);
        //RPM,INTERVALは10回測定後の平均値
        printf("rpm_ave:%4.2f,interval_ave:%4.2f,dir:%2d\n",rpm_ave,interval_ave,dir); 
        printf("r:%.2lf cnt:%4d\n\e[3A",rto,cnt);
    }
}