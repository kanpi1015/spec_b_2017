/*********************************
/2017.9.14 下降phaseの下降距離の処理
/     10.06 
/     10.14 countdown
      10.18 chk
*********************************/

//header fire
#include"safety.h"
//


//window parameter 
const double Target_Alt = 4.0;                                                  //Target altitude [m]
const double Target_des_dist = 3.5;                                             //Target descent dist [m]
const double Target_velo = 4.0;                                                 //Target velocity [m/sec]
//



/***********************
/Elevation current
/brief;
/Phase  0:Ground standby
/      10:Elevation
/      20:Sky standby
/      30:Descent
/      40:Ground return
/
***********************/

int main(){
    init();
    countdown();
    esc_setup();
    wait(5);
    ESC.pulsewidth_us(1500);
    wait(1);
    
    tim_A.start();
    while(1){
        
        Safety();
        PID_control();
        
        if(tim_A.read_ms()>=100){
            tim_A.reset();
            tim_B.start();
    
            switch(Phase){
        
                case 0 : {//ground standby
                    PWM = MOR_N;
                    EMG = emg_ON;
                    //ANGLE = angle_ON;
                    Phase = 10;
                    break;
                    }
                case 10 :{//elevation
                    PWM = PWM_PID;
                    EMG = emg_OFF;
                    //ANGLE = angle_OFF;
                    if(dist_p>=Target_Alt){
                        Phase =20;
                
                        }
                        break;
                    }
                case 20 :{//sky stop
                    PWM = MOR_N;
                    EMG = emg_ON;
                    //ANGLE = angle_ON;
                    tim_C.start();
                    if(tim_C.read_ms()>=5000){
                        Phase = 30;
                        tim_C.reset();
                
                        }
                        break;
                    }
                case 30 :{//descent
                    PWM = PWM_PID;
                    EMG = emg_OFF;
                    //Angle = angle_OFF;
                    if(dist_p <= Target_des_dist){
                        Phase = 40;  
                        }
                        break;
                    }
                case 40 :{//arrival ground
                    PWM = MOR_N;
                    //ANGLE = angle_ON;
                    EMG = emg_ON;
                    break;
                    }           
        }
        ESC.pulsewidth_us(PWM);
        Emg.pulsewidth_us(EMG);
        //Angle.pulsewidth_us(ANGLE);
        USB.printf("%8d\r\n",PWM);
        LOG.printf("%8d\r\n",PWM);
        USB.printf("%8d\r\n",EMG);
        LOG.printf("%8d\r\n",EMG);
        //USB.printf("%8d\r\n",ANGLE);
        //LOG.printf("%8d\r\n",ANGLE);
          }
          else if(tim_B.read_ms()>=100){
              tim_B.reset();
              tim_A.reset();
              }
        }
}

