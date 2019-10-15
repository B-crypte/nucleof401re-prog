#include "mbed.h"
#include "encoder_control.h"

#define CCW 0               //clockwise operation (1bit;0:cw,1:ccw)
#define ZEROPOS 0x00        //programmed zero position (10bit)
#define INDEX 0             //index bit width (1bit)
#define DIVX 0x0            //incremental resolution (2bit;mode_10bit,mode_7bit,mode_5bit)
#define MDX  0x0            //incremental mode (2bit;m1:quadrature,m2:step/dir,m3:motor)

DigitalIn MagINC(PA_12);    //magnitude increase
DigitalIn MagDEC(PA_11);    //magnitude decrease

DigitalOut CS(D6);          //chip select
DigitalOut Prog(PB_15);     //otp program(mode set)
DigitalOut CLK(D5);         //clock(trigger input)
//MDxの定義で変更
//InterruptIn LSB(PA_8,PullUp); //step/dir mode Least Sign Bit
//DigitalIn Dir(PA_9);          //direction of rotation
//DigitalIn Mt_U(PA_8,PullUp);  //U sign(pahse1)
//DigitalIn Mt_V(PA_9,PullUp);  //V sign(phase2)
DigitalIn DO(D4);               //Data Output Serial interface
DigitalIn PWM_LSB(D2);          //PWM LSB in mode3

InterruptIn Index_w(D7,PullUp); //m1,m2:absolute zero pos.,m3:W sign

//global 
int cnt;      //count
bool dir;
unsigned char current;   //記憶値

//counter reset to 0
void Reset_cnt(){
    cnt = 0;
}

//main func.
int main(){
    //interrupt Z phase
    Index_w.rise(Reset_cnt);
    init_enc();
    //main
    while (1) {
        printf("MagInc:%d,MagDec:%d\n",(bool)MagINC,(bool)MagDEC);
        printf("cnt:%4d,dir:%2d\n\e[2A",cnt,dir);
    }
}