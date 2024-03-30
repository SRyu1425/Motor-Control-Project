#ifndef POSITION__H__
#define POSITION__H__
#include <xc.h> // processor SFR definitions
#include <sys/attribs.h> // __ISR macro
#include "NU32DIP.h"


float get_ppgains();
float get_pdgains();
float get_pigains();
void send_ref(float* posns, int num_samp);
void set_pgains(float kp, float ki, float kd);
void posn_sense_startup();
int get_samp();
void set_deg(float d);
void set_EintPrev(void);
float get_deg(void);





#endif // POSITION__H__


