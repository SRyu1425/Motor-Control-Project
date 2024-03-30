#include "currentcontrol.h"
#include <xc.h>
#include "utilities.h"
#include <stdio.h>
#include <stdlib.h>
#include "ina219.h"


volatile int duty; //duty cycle of PWM
static float kp = .07; //hard coded kp, ki values (for ease of testing)
static float ki = .02; //static makes it so that these variables do not affect the other ones that have the same var name
static volatile int ref[100]; //reference mA values of a square wave (+- 200 mA)
volatile int act[100]; //actual current values used to compare against reference in the current control 
volatile float Eint; //integral of the control error
volatile float current; //desired current from the position controller

void set_cgains(float p, float i){
    //setter function to set gains
    kp = p;
    ki = i;
    Eint = 0;
}
void set_Eint(void){
    //reset Eint
    Eint = 0;
}

float get_cpgains(void){
    //getter function to get kp
    return kp;
}

float get_cigains(void){
    //getter function to get ki
    return ki;
}

void set_PWM(int pwm){
    duty = pwm; //set input pwm as the official PWM 
}

void send_current(float curr){
    current = curr; //current sent by motor controller
}

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Controller(void) { // _TIMER_2_VECTOR = 8
    //ISR runs on 5kHz
    //means 5k samples per second
    //reference wave = 100 Hz square wave (+-200 mA)

    switch (get_mode()){
        case 0:
        {
            //IDLE case -> enter brake mode (enable pin must be 0)
            //enable pin is the input duty cycle 
            OC1RS = 0; //stop the PWM
            LATBCLR = 1 << 6; //set output pin to low 
            break;
        }
        case 1:
        {
            //PWM mode
            OC1RS =  abs(duty) * .01 * PR3; //duty cycle is PWM (in dec form) * PR3
            if (duty < 0){
                //backwards direction (phase pin is tied high)
                LATBCLR = 1 << 6;
            } else {
                //forwards direction (phase pin tied low)
                LATBSET = 1 << 6;
            }
            break;
        }
        case 2:
        {
            //itest mode
            static int count = 0; //static var used to tell when to switch from +- 200 mA
            static int ref_val = 200; //starts w/ 200 mA

            if (count == 99){ //reference array only 100 data points
                set_mode(0); //set idle mode
                char message[200]; // message to and from MATLAB
                for (int i=0; i< 100; i++) { // send 100 plot data points to python
                    sprintf(message, "%d %d %d\r\n", ref[i], act[i], 99-i);
                    NU32DIP_WriteUART1(message);
                }
                count = 0; //reset count
            }
            if (count == 25 || count == 50 || count == 75){
                ref_val = ref_val * -1; //switch sign of reference value 
            }
            ref[count] = ref_val; //assign ref value to ref array (to save data)
            act[count] = INA219_read_current(); //get actual current


            int error = ref[count] - act[count]; //error = reference current - actual current
            Eint = Eint + error; //integral of control error update
            float unew = kp * error + ki * Eint; //calculate PI

            //centered around 0
            //cap the duty cycle (only goes from -100 to 100 % of the duty cycle)
            if (unew > 100.0) {
                unew = 100.0;
            } else if (unew < -100.0) {
                unew = -100.0;
            }

            if (unew < 0){
                //backwards direction (phase is tied high) (lat is opposite, so lat should be low)
                LATBCLR = 1 << 6;
            } else {
                //forwards direction (phase tied low)
                LATBSET = 1 << 6;
            }

            
            OC1RS = (unsigned int) ((abs(unew)/100.0) * PR3); //calculates new duty cycle
            count++; //update counter to next index of data arrays
            break;
        }
        case 3:
        {

            //hold mode
            float act_curr = INA219_read_current(); //get actual current


            float error = current - act_curr; //error = reference current - actual current
            Eint = Eint + error; //integral of control error update

            Eint += error;
            if (Eint > 500) {
                Eint = 500;
            } else if (Eint < -500) {
                Eint = -500;
            }          

            float unew = kp * error + ki * Eint;

            //centered around 0

            if (unew > 100.0) {
                unew = 100.0;
            } else if (unew < -100.0) {
                unew = -100.0;
            }

            if (unew < 0){
                //backwards direction (phase is tied high) (lat is opposite, so lat should be low)
                LATBCLR = 1 << 6;
            } else {
                //forwards direction (phase tied low)
                LATBSET = 1 << 6;
            }
            OC1RS = (unsigned int) ((abs(unew)/100.0) * PR3); //calculates new duty cycle
            break;
        }
        case 4:
        {

            
            float act_curr = INA219_read_current(); //get actual current


            float error = current - act_curr; //error = reference current - actual current
            Eint = Eint + error; //integral of control error update

            Eint += error;
            if (Eint > 500) {
                Eint = 500;
            } else if (Eint < -500) {
                Eint = -500;
            }          

            float unew = kp * error + ki * Eint;

            //centered around 0
            if (unew > 100.0) {
                unew = 100.0;
            } else if (unew < -100.0) {
                unew = -100.0;
            }

            if (unew < 0){
                //backwards direction (phase is tied high) (lat is opposite, so lat should be low)
                LATBCLR = 1 << 6;
            } else {
                //forwards direction (phase tied low)
                LATBSET = 1 << 6;
            }
            OC1RS = (unsigned int) ((abs(unew)/100.0) * PR3); //calculates new duty cycle
            break;
        }
    }
    IFS0bits.T2IF = 0; //clear flag
}


void curr_sense_startup(){
    TRISBCLR = 1 << 6; //clear rb6 as output pin
    
    RPB15Rbits.RPB15R = 0b0101; //remappable OC1 to RPB15 pin

    OC1CONbits.OCTSEL = 1;     // Use Timer3 as the clock source for PWM 
    OC1CONbits.OCM = 0b110;    // PWM mode without fault pin

    OC1R = 1000; //(random for now)
    
    //tmr3 = timer used for OC
    //tmr2 = timer used for ISR
    //setup peripheral
    PR2 = 9599;  //5 kHz ISR -> 48M (pic clock speed)/ 5k = 9600 -> 9600 = (PR2 + 1) * 1 -> PR2 = 9599
    PR3 = 2399; // period = (PR3 + 1) * N = 20kHz -> 48M 
    TMR3 = 0; 
    TMR2 = 0; // initialize count to 0
    T2CONbits.TCKPS = 0; // set prescaler to 1
    T3CONbits.TCKPS = 0; // set prescaler to 1
    //T2CONbits.TGATE = 0; // not gated input (the default)
    //T2CONbits.TCS = 0; // PCBLK input (the default)
    IPC2bits.T2IP = 5; // priority is 5 (high relative to motor control)
    IPC2bits.T2IS = 0; // subpriority
    IFS0bits.T2IF = 0; // clear interrupt flag
    IEC0bits.T2IE = 1; //enable interrupt

    T3CONbits.ON = 1; // turn on Timer3
    T2CONbits.ON = 1; // turn on Timer2
    OC1CONbits.ON = 1; // Turn OC1 on
}
