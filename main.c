#include "NU32DIP.h" // config bits, constants, funcs for startup and UART
// include other header files here
#define BUF_SIZE 200
#include "encoder.h"
#include "ina219.h"
#include "utilities.h"
#include "currentcontrol.h"
#include "positioncontrol.h"

int main()
    {
    char buffer[BUF_SIZE];
    NU32DIP_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
    UART2_Startup(); //startup for raspberry pi PICO
    NU32DIP_GREEN = 1; // turn off the LEDs
    NU32DIP_YELLOW = 1; 
    
    enum Mode start = IDLE; //initialize starting mode as idle
    set_mode(start); 
    __builtin_disable_interrupts();
   
    INA219_Startup(); //startup for the current sensor
    curr_sense_startup(); //startup for the current sensor controller
    posn_sense_startup(); //startup for the position sensor controller
    // in future, initialize modules or peripherals here
    __builtin_enable_interrupts();

    NU32DIP_WriteUART1("Start2\r\n");

    while(1)
    {
        NU32DIP_ReadUART1(buffer,BUF_SIZE); //we expect the next character to be a menu command
        NU32DIP_YELLOW = 1; // clear the error LED
        switch (buffer[0]) {              
            case 'b':
            {
                // read current in mA
                //communicate with current sensor over I2C
                float current = INA219_read_current();
                char m[50];
                sprintf(m, "%.2f\r\n", current);
                NU32DIP_WriteUART1(m); //print current to screen
                break;
            }
            case 'q':
            {
                // handle q for quit and set mode to idle
                set_mode(0);
                break;
            }
            case 'c':
            {
                // Pico will print back the encoder count as an integer followed by a newline
                WriteUART2("a"); //send a to pico 
                while(!get_encoder_flag()){}
                set_encoder_flag(0);
                char m[50];
                int p  = get_encoder_count();
                sprintf(m,"%d\r\n",p);
                NU32DIP_WriteUART1(m);
                break;
            }
            case 'd':
            {
                // Pico will print back the encoder count and it will be translated into degree
                WriteUART2("a");
                while(!get_encoder_flag()){}
                set_encoder_flag(0);
                char m[50];
                int p  = get_encoder_count();
                float d = p * (360.0 / 384);
                sprintf(m,"%.2f\r\n",d); //96 lines per rev....(p count % 384) * (360 degrees/384 counts) = posn in degree
                NU32DIP_WriteUART1(m);
                break;
            }
            case 'e':
            {
                // the Pico will reset the encoder count to 0 and will not reply. 
                WriteUART2("b"); //the program that allows this to happen is loaded into the pico
                break;
                
            }
            case 'r':
            {               
                enum Mode motor_mode = get_mode();
                //enum returns a value from 0 to 4. print this in its string representation
                if (motor_mode == 0){
                    NU32DIP_WriteUART1("mode IDLE\r\n");
                } else if (motor_mode == 1){
                    NU32DIP_WriteUART1("mode PWM\r\n");
                } else if (motor_mode == 2){
                    NU32DIP_WriteUART1("mode ITEST\r\n");
                } else if (motor_mode == 3){
                    NU32DIP_WriteUART1("mode HOLD\r\n");
                } else if (motor_mode == 4){
                    NU32DIP_WriteUART1("Mode TRACK\r\n");
                }
                break;
            }

            case 'g':
            {
                //set current gains
                float c_ki, c_kp = 0;
                NU32DIP_ReadUART1(buffer,BUF_SIZE); 
                sscanf(buffer, "%f %f", &c_kp, &c_ki);
                set_cgains(c_kp, c_ki); //call setter function to set gains (from currentcontrol.c)
                break;
            }

            case 'h':
            {
                //get current gains
                float c_kp = get_cpgains(); //call getter functions 
                float c_ki = get_cigains();
                char gains[10];
                sprintf(gains, "Kp gain: %f, Ki gain: %f\r\n", c_kp, c_ki);
                NU32DIP_WriteUART1(gains); //print gains to the screen
                break;
            }
            
            case 'i':
            {
                //set position gains
                float p_ki, p_kp, p_kd = 0;
                NU32DIP_ReadUART1(buffer,BUF_SIZE); 
                sscanf(buffer, "%f %f %f", &p_kp, &p_ki, &p_kd);
                set_pgains(p_kp, p_ki, p_kd); //call setter function to set gains (from positioncontrol.c)
                break;
            }

            case 'j':
            {
                //get position gains
                float p_kp = get_ppgains();
                float p_ki = get_pigains();
                float p_kd = get_pdgains();
                char gains[10];
                sprintf(gains, "Kp gain: %f, Ki gain: %f, Kd gain: %f\r\n", p_kp, p_ki, p_kd); //print to screen
                NU32DIP_WriteUART1(gains);
                break;
            }
            case 'n':
            {
                //load step trajectory
                int N = 0; // number of samples
                NU32DIP_ReadUART1(buffer, BUF_SIZE); // Get number of samples N
                sscanf(buffer, "%d", &N); // Save input number

                float ref[2000]; // Make reference array of adequate size (max = 2000 samples)
                for (int i = 0; i < N; i++) {
                    NU32DIP_ReadUART1(buffer, BUF_SIZE); // Read each sample point
                    float x;
                    sscanf(buffer, "%f", &x); // Save the data point into reference array
                    ref[i] = x;
                }
                send_ref(ref, N); // send pointer to this ref array to positioncontrol.c
                break;
            }
            case 'm':
            {
                //load step trajectory
                int N = 0; //number of samples
                //each plot point is 200 samples (0 0 1 90 = 200), (0 0 1 90 2 90) = 400
                //the list of samples returned will be the angle position at 5ms intervals 
                NU32DIP_ReadUART1(buffer,BUF_SIZE); //first gets number of samples N from the python client file
                sscanf(buffer, "%d", &N); //saves input number
                float ref[2000]; //make reference array of size 2000 (max sample size)
                for (int i = 0; i < N; i++){
                    //loop that goes through the returned position reference array 
                    NU32DIP_ReadUART1(buffer, BUF_SIZE); //read one sample point from the python code
                    float x;
                    sscanf(buffer, "%f", &x); // Save the data point
                    ref[i] = x;                
                }
                send_ref(ref, N); // send pointer to this ref array
                break;
            }
            case 'o':
            {
                //execute trajectory
                set_EintPrev(); //reset eint, eprev (poscontrol.c)
                set_Eint(); //reset eint (currcontrol.c)
                char m[50];
                sprintf(m, "%d\r\n", get_samp());
                NU32DIP_WriteUART1(m); //python expects the number of data points to plot
                set_mode(4); //set in track mode
                break;
            }
            case 'l':
            {
                set_EintPrev();
                set_Eint();
                //go to angle (deg)
                float angle = 0;
                NU32DIP_ReadUART1(buffer,BUF_SIZE);  //get angle input 
                sscanf(buffer, "%f", &angle);
                set_deg(angle);
                set_mode(3); //set in hold mode
                break;
            }

            case 'k':
            {
                //test current gains
                char m[50];
                sprintf(m, "%d\r\n", 100);
                NU32DIP_WriteUART1(m); //python expects the number of data points to plot
                set_mode(2); //set in itest mode
                break;
            }

            case 'f':
            {
                //Set PWM (-100 to 100)
                int PWM;
                NU32DIP_ReadUART1(buffer,BUF_SIZE); //get pwm from user
                sscanf(buffer, "%d", &PWM); //put input number into pwm variable
                set_PWM(PWM); //sets the pwm rate.
                set_mode(1); //switches to PWM mode
                break;
            }
            case 'p':
            {
                //Unpower the motor
                set_mode(0); //set mode to idle
                break;
            }
            default:
            {                
                NU32DIP_WriteUART1("fail\r\n");
                NU32DIP_YELLOW = 0; // turn on LED2 to indicate an error
                break;
            }
        }
    }
    return 0;
}