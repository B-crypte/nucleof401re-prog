#include "mbed.h"
#include "motor_control.h"
#include "encoder_control.h"

PwmOut Mt1(D11);     //motor Input1
PwmOut Mt2(D10);     //motor Input2
Ticker  Update_mtr;  //

static float en1 = 0.0f;
static float en2 = 0.0f;
static float MVn = 0.0f;
static float current_spd = 0.0f;
static float pwm_rate = 0.0f;
static float goal_spd;    //[rad/s]
static float x[7]={0};
static float c=0;
static float work1[2] = { 0 };

//初期化
void init_mtr(void){
    en1 = 0.0f;
    en2 = 0.0f;
    MVn = 0.0f;
    current_spd = 0.0f;
    pwm_rate = 0.0f;
    Mt1.write(1.0f);
    Mt2.write(1.0f);
    Mt1.period_ms(1);
    Mt2.period_ms(1);
    set_goal_rpm(60.0f);
}
//PWM設定
void write_pwm_mtr(int no,float per){
    switch(no) {
        case 0:
            Mt1.write(per);
            break;
        case 1:
            Mt2.write(per);
            break;
        default:
        {}
    }
}
//PID制御用
void pid_ctr_mtr(float per){
    if(per > 1.0f) per = 1.0f;
    if(per < 0.0f) per = 0.0f;
    Mt1.write(1.0f);
    Mt2.write(1.0f-per);
}
//モータ動作状態読み取り
float readstate_mtr(int no_mtr){
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
void start_mtr(){
    //第二引数，時間指定を変数，マクロですると動作がおかしくなる
    Update_mtr.attach(&motor_pid,0.01f);
    //Update_mtr.attach(&spd_ctr_pid,0.01f);
}
void stop_mtr(){
    write_pwm_mtr(1,1.0f);
    Update_mtr.detach();
}
void change_spd(float set_spd){
    goal_spd = set_spd;
}
//RPMから角周波数[rad/s]を求め，目標値にセットする
void set_goal_rpm(float s_rpm){
    goal_spd = s_rpm;
}
void ctr_spd(float add){
    goal_spd += add;
    if(goal_spd < 0.0f)
        goal_spd = 0.0f;
}
//モータの角速度制御
void motor_pid(void){
    float en,ep,ei,ed;      //各偏差
    float rate_reduction = 38.2f;   //減速比
    float max_spd = 20000.0f;       //PWMレートが最高速度で1.0になるようにする

    //関数呼び出し：角速度の計算[rad/s]
    current_spd = mesu_rot_speed();
    reset_enc_cnt();
    //偏差=目標値[rpm]-現在速度[rad/s→rpm]
    en = goal_spd - (current_spd*60.0f/(2.0f*PI));
    en *= rate_reduction; //軸の回転数[rpm*減衰比]
    //PID制御による制御量の決定
    ep = en - en1;
    ei = dT * (en+en1)/2.0f;
    ed = (en-2*en1+en2)/dT;
    //操作量の決定
    MVn += Kp * (ep + (ei/Ti)+Td*ed);
    //MVn = Kp * (ep + ei/Ti);
    //PWMの比率で結果を反映(最大角速度：-[rad/s])
    pwm_rate = MVn/max_spd;
    pwm_rate = limit(pwm_rate,0.0f,1.0f);
    pid_ctr_mtr(pwm_rate);
    //偏差を記録
    en2 = en1;
    en1 = en;
    //デバッグ用
    x[0]=en;
    x[1]=ep;
    x[2]=ei;
    x[3]=ed;
    x[4]=MVn;
    x[5]=current_spd;
}
//PID制御関数(不安定になる)
void spd_ctr_pid(){
    float rate_reduction = 38.2f;    //減速比
    float max_spd = 17800.0f;        //PWMレートが最高速度で1.0になるようにする

    x[6]=mesu_rot_speed();  //現在速度[rad/s]
    reset_enc_cnt();        //カウンタリセット
    c=x[6]*60.0f/(2.0f*PI); //rad/s→rpmに変換
    x[0]=goal_spd;      //目標値設定
    x[1]=x[0]-c;        //偏差
    x[2]=pid(x[1],Kp,Ki,Kd,work1,dT);   //pid
    x[3]=x[2]*rate_reduction;
    x[4]=x[3]/max_spd;                  //rpm→pwm rate
    x[5]=limit(x[4],0.0f,1.0f);         //比率を範囲内に収める
    pwm_rate=x[5];
    pid_ctr_mtr(pwm_rate);              //pwm出力
}
//デューティ比率（操作値）
float read_ratio(){
    return pwm_rate;
}
//第一引数が範囲内を超えないように調整する関数
float limit(float target,float min,float max){
    float buf = target;
    if(buf < min) buf = min;
    if(buf > max) buf = max;
    return buf;
}
float pid(float xin, float kp, float ki, float kd, float* work,float dt) {
	float a, b, c, xout;

	a = xin;
	b = work[1] + (xin + work[0]) / 2.0 * dt;
	c = (xin - work[0]) / dt;
	work[0] = xin;
	work[1] = b;

	xout = a * kp + b * ki + c * kd;
	return (xout);
}
void pid_value_disp(){
    for(int i = 0;i < 7;i++){
        printf("x[%d]:%.3f,",i,x[i]);
    }
    printf("c:%.3f\n",c);
}