# Motor-Control Project
This repository contains an advanced motor control system designed to demonstrate complex programming, PID control, mechatronics knowledge, and microcontroller interfacing. The project illustrates the integration of hardware and software to perform precise motor control tasks using a PIC32 microcontroller, a Raspberry Pi Pico, an INA219 current sensor, an H-bridge for motor driving, and a battery as a power source.

## System Architecture
The system operates as follows: The USBtoUART chip communicates with the PIC32 microcontroller, which communicates with the Raspberry Pi Pico. The Pico is responsible for reading the encoder attached to the motor, which informs the system of the motor's position. The INA219 current sensor monitors the current powering the motor (proportional to the torque) and feeds data back for current control. The H-bridge receives commands from the PIC to control the direction and speed of the motor based on the PID feedback loops, all powered by a connected battery. In total there are 2 control loops, an inner current control loop running at a higher frequency, and outer position control loop running at a lower frequency.

## Features
- Dual PID Control Loops: Implements both position and current control loops for precise motor operation.
- UART Communication: Leverages UART protocol for inter-device communication between PIC32 and Raspberry Pi Pico.
- Python Integration: Uses Python scripts for data communication with the C program and for graphically plotting motor trajectories.
- Terminal-based UI: Provides a user interface through the terminal to interact with the numerous motor control functionalities.

## Testing and Validation
The motor's performance was tested against predefined trajectories, demonstrating it can follow the paths with the proper PID gains. Here are some plots below:


![positionControl](https://github.com/SRyu1425/Motor-Control-Project/assets/142364914/59c39707-8923-42ea-af3e-eda1d8669ac1)
![positionControl_square](https://github.com/SRyu1425/Motor-Control-Project/assets/142364914/59cfc774-f4ee-4458-83b2-49302820c1b1)
