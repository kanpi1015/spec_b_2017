#include"mbed.h"
#include"QEI.h"

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
//#define servo_pin p26
//#define startpin pin30

Serial LOG(p9,p10);
Serial USB(USBTX,USBRX);                                                        //Serial comunication's pin with pc
PwmOut ESC(escpin);                                                             //Esc'pin is p25
PwmOut Emg(emg_pin);                                                            //Clutch brake is p22
//PwmOut Angle(servo_pin);
DigitalIn sw1(sw1_pin);                                                         //switch pin_1
DigitalIn sw2(sw2_pin);                                                         //swintch pin_2
QEI ENC_d(penc_A,penc_B,NC,300, QEI::X4_ENCODING);                                   //drive pin is p15 and pin16
QEI ENC_p(denc_A,denc_B,NC,300, QEI::X4_ENCODING);                                   //passive pin is p17 and p18
//

//
extern const double Target_velo;
const int MOR_N = 1500;
const int emg_ON = 1;
const int emg_OFF = 20000;
//const int angle_ON = 2000;
//const int angle_OFF = 1500;
const double wheel_dia_p = 0.3;
const double wheel_dia_d = 0.5;                                            //wheel diameter(m)
const double PI = 3.141592;
const double dGR = 2.0;
const double pGR = 2.0;
int Phase = 0;
int esc_type = 1;                                                               //TMM = 0 HV-160 = 1
//


//window parameter
double PWM = 0.0,ANGLE = 0.0, EMG = 0.0;                                        //out put
double p_enc = 0.0,d_enc = 0.0;                                                 //encording
double unit_dist_p = 0.0, velo_p = 0.0, dist_p = 0.0;                           //passive unit parameter [m] [m/s]  [m]
double unit_dist_d = 0.0, velo_d = 0.0, dist_d = 0.0;                           //drive unit parameter
double fall_dist = 0.0;                                                         //for Fall_safty[m]
//


//Timer
Timer Runtime, tim_A,tim_B,tim_C,tim_Deta;                                         //TIMER Runtime Period1 Period2 Waittime Detarecording


//Sample time
const double chk_tim = 0.1;                                                     //Timer check samlpe time
//


//PID parameter
double PWM_PID = 0.0;
double e = 0.0, e_old1 = 0.0, e_integral = 0.0;     //Current_devietion Old_deviation Sum of deviations
const double kp = 50.0;                                                          //Gein p
const double ki = 0.0;                                                          //Gein i
const double kd = 0.0;                                                          //Gein d
double PWM_p,PWM_i,PWM_d,PWM_U;
const int PWM_max_limit = 2000;                                                 //PWM max limit  for moter safety
const int PWM_min_limit = 1550;                                                 //PWM minimum limit
//


//Private function
void init(void);                                                                //Initialize
void esc_setup(void);                                                           //Esc_setup proces
void unit_calculate(void);                                                      //Unit's sate calculation
void PID_control(void);                                                         //PID control processing
void DetaRec(void);                                                             //Deta recording processing
//

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
    //Angle.period_us(2000);
    USB.printf("\n\rclimer_initialize");
    LOG.printf("\n\rclimer_initialize");



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
    wait(1);
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
    unit_dist_d = wheel_dia_d*(d_enc / 1200)*PI*dGR;
    unit_dist_p = wheel_dia_p*(p_enc / 1200)*PI*pGR;                                //enc_n 1200
    ENC_d.reset();
    ENC_p.reset();
    dist_d += unit_dist_d;                                                      //current altitude drive [m]
    dist_p += unit_dist_p;                                                      //current altitude passive [m]
    velo_d = fabs(unit_dist_d)/0.1;
    velo_p = fabs(unit_dist_p)/0.1;                                              //samp_t 0.1

}

/*************************************
* PID_control
* Author :Yuki Kitajima
* Brief :calculate PWM pluses
* error :error_old error_old2 error_old3にて積分補償を行い精度向上を目指す
*************************************/
void PID_control(void)
{
    unit_calculate();                                                             //get velo_d and velo_p[m/s]

    e_old1 = e;
    e = Target_velo - velo_p;
    e_integral = e + e_old1;
    PWM_p = kp*e;
    PWM_i = ki*e_integral;
    PWM_d = kd*(e - e_old1)/0.1;                                                 //samp_t 0.1[sec]

    PWM_U = PWM_p + PWM_i + PWM_d;

    PWM_PID = MOR_N + PWM_U;

    if(PWM_U > PWM_max_limit - 1500)PWM_U = PWM_max_limit - 1500;               //PWM max limit
    if(PWM_U < PWM_min_limit - 1500)PWM_U = PWM_min_limit - 1500;               //PWM min limit

}

/*******************************
/ Detarec
/ Author :Yuki Kitajima
/ Brief :Deta recording
******************************/
void Detarec()
{
    tim_Deta.start();
    while(1){
        if(tim_Deta.read_ms()>=100){
            tim_Deta.reset();
            tim_Deta.start();

            USB.printf("%.2lf%8d%8.0d%8.2lf%8.2lf%8.2lf%8.2lf%8d\r\n", Runtime, PWM, EMG, velo_d, velo_p, dist_d, dist_p, Phase);
            LOG.printf("%.2lf%8d%8.0d%8.2lf%8.2lf%8.2lf%8.2lf%8d\r\n", Runtime, PWM, EMG, velo_d, velo_p, dist_d, dist_p, Phase);
            }
        }
}
