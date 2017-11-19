#include"config.h"

//timer
Timer timSL,timFL,timST,tim_sw1,tim_sw2,timENC;

/**************************
/
/
/
*************************/
extern double velo_d;
extern double velo_p;
extern double unit_dist_d;
extern double unit_dist_p;
extern double fall_dist;
extern double dist_d;
extern double dist_p;
extern int Phase;



//private function
void Safety(void);
void slip_safety(void);
void fall_safety(void);
void stuck_safety(void);
void sw_chk(void);
void sw1_chk(void);
void sw2_chk(void);
void encorder_error(void);


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
    sw_chk;
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
    if((velo_p<=0.3)&&(dist_p>=1.0)&&(Phase == 30)&&(Phase == 10)){
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

void sw1_chk(void)
{
    if(sw1.read() == 1.0){
        tim_sw1.start();
        
        if(tim_sw1.read()>=chk_tim){                                             //check time 0.1[sec]
            tim_sw1.reset();
            PWM = MOR_N;
            EMG = emg_ON;
            //ANGLE = angle_ON;
            Phase = 20;
            }
        }
        else {
            tim_sw1.reset();
            }   
}

void sw2_chk(void)
{
    if(((sw2.read() == 1.0)&&(Phase = 30)&&(Phase = 10))){                      //phase 10 = Emergency stop
        tim_sw2.start();
        
        if(tim_sw2.read()>chk_tim){                                             //check time 0.1[sec]
            PWM = MOR_N;
            EMG = emg_ON;
            //ANGLE = angle_ON;
            Phase = 40;                                                         //Safety 上空停止
            
            }
        }
        else {
            tim_sw2.reset();
            }
}

void encorder_error(void)
{
    if((unit_dist_p==0)||(unit_dist_d==0)&&(Phase < 20)){
        timENC.start();
        if(timENC.read()>=3.0){
            Phase = 20;
            USB.printf("\n\rencorder_error_detected");
            LOG.printf("\n\rencorder_error_detected");
            timENC.reset();
            }
            else{
                timENC.reset();
                }
        }   
}