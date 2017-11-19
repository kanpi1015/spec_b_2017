/*********************************
/2017.9.14 下降phaseの下降距離の処理
/     10.06
/     10.14 countdown
      10.18 chk
*********************************/


//header fire
#include"mbed.h"
#include"QEI.h"
//


//Pin designation
#define Logertx p9                                                              //Openlog
#define Logerrx p10                                                             //
#define sw1_pin p13                                                             //Upper switch pin
#define sw2_pin p14                                                             //Lower switch pin
#define penc_A p15                                                              //Passive encorder pin A
#define penc_B p16                                                              //Passive encorder pin B
#define denc_A p17                                                              //Drive encorder pin A
#define denc_B p18                                                              //Drive encorder pin B
#define escpin p25                                                              //Esc
#define emg_pin p21                                                             //Clutch brake


Serial LOG(p9,p10);
Serial USB(USBTX,USBRX);                                                        //Serial comunication's pin with pc
PwmOut ESC(escpin);                                                             //Esc'pin is p25
PwmOut Emg(emg_pin);                                                            //Clutch brake is p22
DigitalIn sw1(sw1_pin);                                                         //switch pin_1
DigitalIn sw2(sw2_pin);                                                         //swintch pin_2
QEI ENC_d(denc_A,denc_B,NC,300, QEI::X4_ENCODING);                                   //drive pin is p15 and pin16
QEI ENC_p(penc_A,penc_B,NC,300, QEI::X4_ENCODING);                                   //passive pin is p17 and p18
//

#define MOR_N 1000
#define PWM_samp 1350
#define PWM_accel 10

int PWM = MOR_N;
const int emg_ON = 1;
const int emg_OFF = 20000;
const double wheel_dia_p = 0.03;
const double wheel_dia_d = 0.05;                                            //wheel diameter(m)
const double PI = 3.141592;
const double dGR = 2.05263158;
const double pGR = 2.0;
int Phase = 0;
int esc_type = 1;                                                               //TMM = 0 HV-160 = 1
//


//window parameter
const double Target_Alt = 20.0;                                                  //Target altitude [m]
const double Target_des_dist = 1.0;                                             //Target descent dist [m]
const float Target_velo = 1.0;                                                 //Target velocity [m/sec]
int EMG = 0;                                                                    //clutch brake defolt pwm

double p_enc = 0.0,d_enc = 0.0;                                                 //encording
double unit_dist_p = 0.0, velo_p = 0.0, dist_p = 0.0;                           //passive unit parameter [m] [m/s]  [m]
double unit_dist_d = 0.0, velo_d = 0.0, dist_d = 0.0;                           //drive unit parameter
double fall_dist = 0.0;                                                         //for Fall_safty[m]
//


//Timer
Timer Runtime,tim_A,tim_B,tim_C,timSL,timFL,timST,tim_sw1,tim_sw2,timENC;                                         //TIMER Runtime Period1 Period2 Waittime
//


//Sample time
const double chk_tim = 0.1;                                                     //Timer check samlpe time
//

//PID parameter
double e = 0.0, e_old1 = 0.0, e_integral = 0.0;                                 //Current_devietion Old_deviation Sum of deviations
const double kp = 60.0;                                                          //Gein p
const double ki = 30.0;                                                          //Gein i
const double kd = 0.0;                                                          //Gein d
double PWM_p,PWM_i,PWM_d,PWM_U;
const int PWM_max_limit = 1600;                                                 //PWM max limit  for moter safety
const int PWM_min_limit = 1000;                                                  //PWM minimum limit
//


//Private function
void init(void);                                                                //Initialize
void countdown(void);
void esc_setup(void);                                                           //Esc_setup proces
void unit_calculate(void);                                                      //Unit's sate calculation
void PID_control(void);                                                         //PID control processing
void Safety(void);
void slip_safety(void);
void fall_safety(void);
void stuck_safety(void);
void sw_chk(void);
void sw1_chk(void);
void sw2_chk(void);
void encorder_error(void);
//


/***********************
/Elevation current
/brief;
/Phase  0:Ground standby
/       1:Accelerate
/      10:Elevation
/      20:Sky standby
/      30:Descent
/      40:Ground return
/
***********************/

int main(){
    init();
    esc_setup();
    countdown();

    tim_A.start();
    Runtime.start();
    while(1){

        //Safety();

        if(tim_A.read_ms()>=100){
            tim_A.reset();
            tim_B.start();

            unit_calculate();
            PID_control();

            switch(Phase){

                case 0 :{//ground standby
                    //PWM += PWM_accel;
                    EMG = emg_OFF;
                    USB.printf("Start\r\n");
                    USB.printf("time\t  PWM\tEMG\tvelo_d\tvelo_p\tdist_d\tdist_p\tPhase\tTarget_v  PWM_U\te  e_old1 unit_dist_d\r\n");
                    //LOG.printf("time\t  PWM\tEMG\tvelo_d\tvelo_p\tdist_d\tdist_p\tPhase\tTarget_v  d_enc  p_enc\r\n");
                    //USB.printf("time\t\r\n");
                    Phase = 10;
                    break;
                    }
                case 1 :{//accel pahse
                    PWM += PWM_accel;
                    EMG = emg_OFF;
                    if (PWM >= PWM_samp){
                        PWM = PWM_samp;
                        Phase = 10;                                             //PWM_samp is test palameter 　
                               }                                                //Accelerate to a reasonable speed
                   break;
                   }
                case 10 :{//elevation
                    PWM += PWM_U;
                    EMG = emg_OFF;
                    if(dist_d>=Target_Alt){
                        Phase =20;
                        }
                        break;
                    }
                case 20 :{//sky stop
                    PWM = MOR_N;
                    EMG = emg_ON;
                    tim_C.start();
                    if(tim_C.read_ms()>=5000){
                        Phase = 30;
                        tim_C.reset();

                        }
                        break;
                    }
                case 30 :{//descent
                    PWM = MOR_N;
                    EMG = emg_OFF;
                    if(dist_d <= Target_des_dist){
                        Phase = 40;
                        }
                        break;
                    }
                case 40 :{//arrival ground
                    PWM = MOR_N;
                    EMG = emg_ON;
                    break;
                    }
                case 50 :{
                    PWM = MOR_N;
                    EMG = emg_OFF;
                    break;
                }
        }
        ESC.pulsewidth_us(PWM);
        Emg.pulsewidth_us(EMG);

          }
          else if(tim_B.read_ms()>=100){
              tim_A.start();
              tim_B.reset();
              USB.printf("%.2lf\t%8d%8d%8.4lf%8.2lf%8.2lf%8.2lf%8d%8.1lf%8.2lf%8.2lf%8.2lf%8.4lf\r\n", Runtime.read() , PWM, EMG, velo_d, velo_p, dist_d, dist_p, Phase, Target_velo, PWM_U, e, e_old1, unit_dist_d);
              //LOG.printf("%.2lf\t%8d%8d%8.4lf%8.2lf%8.2lf%8.2lf%8d%8.1lf%8.2lf%8.2lf%8.2lf%8.4lf\r\n", Runtime.read() , PWM, EMG, velo_d, velo_p, dist_d, dist_p, Phase, Target_velo, PWM_U, e, e_old1, unit_dist_d);
              //USB.printf("%.2lf\t\r\n", Runtime.read());
              }

        }
}


/*************************************************
/ initialize
/ Author :Yuki Kitajima
/ Brief :set boudrate ,swicth 1.2 ,ESC.EMG period
**************************************************/
void init(void)
{
    USB.baud(115200);                                                           //baudrate 115200
    LOG.baud(115200);
    sw1.mode(PullUp);
    sw2.mode(PullUp);
    ESC.period_us(2000);
    Emg.period_us(20000);
    USB.printf("\n\rclimer_initialize\r\n");
    LOG.printf("\n\rclimer_initialize\r\n");
    USB.printf("\r\nTarget_Altitude:%.1lf\r\nTarget_velocity:%.1f\r\nTarget_destance:%.1lf\r\nAccel_Rate:%d\r\n",Target_Alt,Target_velo,Target_des_dist,PWM_accel);
    LOG.printf("\r\nTarget_Altitude:%.1lf\r\nTarget_velocity:%.1f\r\nTarget_destance:%.1lf\r\nAccel_Rate:%d\r\n",Target_Alt,Target_velo,Target_des_dist,PWM_accel);
    wait(2);
}

/********************************
/ countdown
/ Author :Yuki Kitajima
/ Brief :countdown 5[sec]
*********************************/
void countdown(void)
{
    int i;
    for(i = 5; i > 0; i--){

        USB.printf("COUNT_DOWN:%d\r\n", i);
        LOG.printf("COUNT_DOWN:%d\r\n", i);
        wait(1);

        }
}

/********************************
/ esc_setup
/ Author :Yuki Kitajima
/ Brief :Set up ESC type
********************************/
void esc_setup(void)
{
    USB.printf("ESC_check\r\n");
    LOG.printf("ESC_check\r\n");
    wait(0.1);
    Emg.pulsewidth_us(emg_ON);
    switch(esc_type){
        case 0:
        //TMM
        USB.printf("ESC::TMM40063\n");
        LOG.printf("ESC::TMM40063\n");
        USB.printf("Set_Midlle_duty\n");
        LOG.printf("Set_Midlle_duty\n");
        ESC.pulsewidth_us(1500);
        wait(5);
        USB.printf("Set_High_duty\n");
        LOG.printf("Set_high_duty\n");
        ESC.pulsewidth_us(1950);
        wait(5);
        USB.printf("Set_Low_duty\n");
        LOG.printf("Set_Low_duty\n");
        ESC.pulsewidth_us(1000);
        wait(5);
        USB.printf("Setup_over\n");
        LOG.printf("Setup_over\n");
        wait(5);

        case 1:
        //HV-160
         USB.printf("ESC::HV-160\r\n");
         LOG.printf("ESC::HV-160\r\n");
         ESC.pulsewidth_us(1000);
         wait(1);
         USB.printf("Set-up Over!!!\r\n");
         LOG.printf("Set-up Over!!!\r\n");
         break;
        }
}

/***************************************
/ unit_calculate
/ Author :Yuki Kitajima
/ Brief:calculate unit diat and velocity
***************************************/
void unit_calculate(void)
{
    d_enc = (double)ENC_d.getPulses();                                          //drive encoder pulses [double]
    p_enc = (double)ENC_p.getPulses();                                          //passive encoder pulses [double]
    unit_dist_d = (wheel_dia_d)*(d_enc / 1200)*PI*dGR;
    unit_dist_p = (wheel_dia_p)*(p_enc / 1200)*PI*pGR;                          //enc_n 1200
    ENC_d.reset();
    ENC_p.reset();
    dist_d += unit_dist_d;                                                      //current altitude drive [m]
    dist_p += unit_dist_p;                                                      //current altitude passive [m]
    velo_d = fabs(unit_dist_d)/0.1;
    velo_p = fabs(unit_dist_p)/0.1;                                             //samp_t 0.1
}

/*************************************
* PID_control
* Author :Yuki Kitajima
* Brief :calculate PWM pluses
* error :error_old error_old2 error_old3にて積分補償を行い精度向上を目指す
*************************************/
void PID_control(void)
{

    e_old1 = e;
    e = Target_velo - velo_d;
    e_integral = (e + e_old1)*0.1/2;
    PWM_p = kp*e;
    PWM_i = ki*e_integral;
    PWM_d = kd*(e - e_old1)/0.1;                                                 //samp_t 0.1[sec]

    PWM_U = PWM_p + PWM_i + PWM_d;

    if(PWM > PWM_max_limit)PWM = PWM_max_limit;               //PWM max limit
    if(PWM < PWM_min_limit)PWM = PWM_min_limit;               //PWM min limit

}

/********************
/ Safety
/ Auther :Yuki Kitajima
/ Brif :
********************/
void Safety()
{
    slip_safety();
    fall_safety();
    stuck_safety();
    sw_chk();
    encorder_error();
}

/**************************
/ slip_safety
/ Author :Yuki Kitajima
/ Brief :
***************************/
void slip_safety()
{
    if((velo_d-velo_p >= 0.5)||(velo_p-velo_d >= 0.5)&&(Phase == 10)){
        timSL.start();
        if(timSL.read()>1.0){
            Phase = 20;
            USB.printf("Slip_Detected!\r\n");
            LOG.printf("Slip_Detected!\r\n");
            }
        } else {
            timSL.reset();
            }
}

/************************
/ fall_safety
/ Auther :Yuki Kitajima
/ Brief :
************************/
void fall_safety()
{
    if(((Phase < 30)&&(unit_dist_p<0))){
        timFL.start();
        fall_dist += unit_dist_p;
        if(fall_dist<-0.5){
            EMG = emg_ON;
            Phase = 20;
            USB.printf("Fall_Detected!\r\n");
            LOG.printf("Fall_Detected!\r\n");
            }
        } else {
            timFL.reset();
            fall_dist = 0.0;
            }
}

/*******************
/auther:Yuki Kitajima
/date:
/brief:
*******************/

void stuck_safety()
{
    if((velo_p<=0.3)&&(dist_p>=1.0)&&((Phase > 20)&&(Phase != 40)&&(Phase != 50))){
        timST.start();
        if(timST.read()>=3.0){
            Phase = 20;
            USB.printf("Stuck_Detected!\r\n");
            LOG.printf("Stuck_Detected!\r\n");
            }
        } else {
            timST.reset();

            }
}

/******************
/auther:Yuki Kitajima
/date:
/brief:
******************/

void sw_chk(void)
{
    sw1_chk();
    sw2_chk();
}


/***************************************
/auther: kitajima
/date:
/brief:検知後上空待機後下降シーケンスへ移行
****************************************/
void sw1_chk(void)//Top switch check sequence
{
    if(sw1.read() == 1.0){
        tim_sw1.start();

        if(tim_sw1.read()>=chk_tim){                                             //check time 0.1[sec]
            tim_sw1.reset();
            PWM = MOR_N;
            EMG = emg_ON;
            Phase = 20;
            USB.printf("Top_switch detected!\r\n");
            LOG.printf("Top_switch detected!\r\n");
            }
        }
        else {
            tim_sw1.reset();
            }
}
/******************************
/auther: kitajima
/date:
/brief:検知後完全に停止
******************************/
void sw2_chk(void)//End switch check sequence
{
    if((sw2.read() == 1.0)&&(Phase <= 30)){                                     //phase 10 = Emergency stop　30 = collision stop
        tim_sw2.start();

        if(tim_sw2.read()>chk_tim){                                             //check time 0.1[sec]
            PWM = MOR_N;
            EMG = emg_ON;
            Phase = 40;                                                         //Safety 上空停止
            USB.printf("End_switch detected!\r\n");
            LOG.printf("End_switch detected!\r\n");
            }
        }
        else {
            tim_sw2.reset();
            }
}

void encorder_error(void)
{
    if((unit_dist_p==0)||(unit_dist_d==0)&&(Phase <= 10)){
        timENC.start();
        if(timENC.read()>=3.0){
            PWM = MOR_N;
            EMG = emg_ON;
            Phase = 20;
            USB.printf("\n\rencorder_error_detected\r\n");
            LOG.printf("\n\rencorder_error_detected\r\n");
            timENC.reset();
            }
            else{
                timENC.reset();
                }
        }
}
