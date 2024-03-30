#include "positioncontrol.h"
#include <xc.h>
#include "utilities.h"
#include <stdio.h>
#include "currentcontrol.h"
#include <stdlib.h>
#include "encoder.h"


//cubic wave: 30,0,1
//square wave: 100,0,2000
static volatile float kp;
static volatile float ki;
static volatile float kd;

static volatile float act[2000]; //actual degree values used to compare against reference in the position control
static volatile float Eint; //integral of the control error
static volatile int eprev; //used for PID (d portion)
static volatile float* ref_posn; //pointer to the reference array (sent by main.c)
static volatile float desired_deg; //desired position input by user
static volatile int N; //number of samples of reference array

void set_pgains(float p, float i, float d){
    //setter function to set gains
    kp = p;
    ki = i;
    kd = d;
    Eint = 0;
    eprev = 0;
}
void set_EintPrev(void){
    //reset eint, eprev
    Eint = 0;
    eprev = 0;
}
float get_ppgains(void){
    //getter function to get kp
    return kp;
}
float get_pigains(void){
    //getter function to get ki
    return ki;
}
float get_pdgains(void){
    //getter function to get kd
    return kd;
}
void send_ref(float* posns, int num_samp){
    //main.c uses this to send a pointer to the ref array and the number of samples
    ref_posn = posns;
    N = num_samp;
}
int get_samp(void){
    //get the number of samples
    return N;
}
void set_deg(float d){
    //set the desired degree
    desired_deg = d;
}
float get_deg(void){
    //get the desired degree
    return desired_deg;
}

void __ISR(16, IPL3SOFT) Controller2(void) { 
    //ISR runs on 200 Hz
    //200 samples per sec
    switch (get_mode()){
        case 3:
        {            
            //hold mode
            //read the encoder
            WriteUART2("a"); //send a to pico 
            while(!get_encoder_flag()){}
            set_encoder_flag(0);
            int p  = get_encoder_count();
            float actual = p * (360.0 / 384); //get encoder count in degree

            float error = desired_deg - actual; //error = reference - actual
            float edot = (error - eprev);
            Eint = Eint + error; //integral of control error update

            //anti integral windup
            if (Eint > 1000) {
                Eint = 1000;
            } else if (Eint < -1000) {
                Eint = -1000;
            }

            //pid
            float unew = kp * error + ki * Eint + kd * edot;
            eprev = error;

            //max stall current is 1300 mA
            if (unew > 1300.0) {
                unew = 1300.0;
            } else if (unew < -1300.0) {
                unew = -1300.0;
            }

            send_current(unew); //send the desired current (output of motion PID) to the current PI controller
            break;

        }
        case 4:
        {
            static int count = 0; //index counter starts at 0

            WriteUART2("a"); //send a to pico
            while(!get_encoder_flag()){}
            set_encoder_flag(0);
            int p  = get_encoder_count();
            act[count] = p * (360.0 / 384); //get encoder count in degree
    
            if (count == N){ //if on last ref posn go into IDLE
                desired_deg = ref_posn[count]; //set last known ref posn to HOLD
                char message[100];
                for (int i=0; i< N; i++) { // send plot data to python
                    //send each data point (real and reference)                    
                    sprintf(message, "%.2f %.2f\r\n", ref_posn[i], act[i]);
                    NU32DIP_WriteUART1(message);
                }
                //set idle
                set_mode(0);
                count = 0; //reset count
            }
            
            float error = ref_posn[count] - act[count]; //error = reference - actual
            float edot = (error - eprev);

            Eint += error;
            //anti integral windup
            if (Eint > 1000) {
                Eint = 1000;
            } else if (Eint < -1000) {
                Eint = -1000;
            }               

            //PID
            float unew = kp * error + ki * Eint + kd * edot;
            eprev = error;

            if (unew > 1300.0) {
                unew = 1300.0;
            } else if (unew < -1300.0) {
                unew = -1300.0;
            }

            send_current(unew); //send the desired current to the current PI controller
            
            count++; //update counter to next index of data arrays
            break;

        }
    }
    IFS0bits.T4IF = 0; //clear flag
}




void posn_sense_startup(){

    PR4 = 58999;  //200 Hz ISR -> 48M/ 200 = 240,000 -> 240,000 = (PR2 + 1) * 4 -> PR4 = 58,999
    TMR4 = 0; // initialize count to 0
    T4CONbits.TCKPS = 2; // set prescaler to 4
    IPC4bits.T4IP = 3; // priority is 3
    IPC4bits.T4IS = 0; // subpriority
    IFS0bits.T4IF = 0; // INT step 5: clear interrupt flag
    IEC0bits.T4IE = 1; // INT step 6: enable interrupt
    T4CONbits.ON = 1; // turn on Timer4
}









