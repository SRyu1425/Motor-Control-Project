#ifndef UTILITIES__H__
#define UTILITIES__H__

#include <xc.h> // processor SFR definitions
#include "NU32DIP.h" // config bits, constants, funcs for startup and UART


//custom data type to indicate mode of the motor
enum Mode {
  IDLE, 
  PWM, 
  ITEST, 
  HOLD, 
  TRACK
};

enum Mode get_mode(void);
void set_mode(enum Mode new_mode);

#endif // UTILITIES__H__
