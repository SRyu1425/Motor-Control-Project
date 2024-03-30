#include "utilities.h"

volatile enum Mode curr_mode; //mode of the motor currently (custom variable)

//getter function
enum Mode get_mode(void){
    return curr_mode;
}

//setter function
void set_mode(enum Mode new_mode){
    curr_mode = new_mode;
}

