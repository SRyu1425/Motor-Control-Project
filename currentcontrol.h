#ifndef CURRENT__H__
#define CURRENT__H__
#include <xc.h> // processor SFR definitions
#include <sys/attribs.h> // __ISR macro

#include "NU32DIP.h"

void curr_sense_startup();
void set_PWM(int pwm);
float get_cpgains();
float get_cigains();
void set_cgains(float kp, float ki);
void send_current(float curr);
void set_Eint(void);


#endif // CURRENT__H__
