# chapter 28 in python

# sudo apt-get install python3-pip
# python3 -m pip install pyserial
# sudo apt-get install python3-matplotlib

import serial
ser = serial.Serial('COM7',230400)
print('Opening port: ')
print(ser.name)

from genref import genRef


import matplotlib.pyplot as plt 
from statistics import mean 
def read_plot_matrix(): #ITEST (ints)
    n_str = ser.read_until(b'\n');  # get the number of data points to receive from the pico
    n_int = int(n_str) # turn it into an int
    print('Data length = ' + str(n_int)) #print how many data sample points there are
    ref = []
    data = []
    data_received = 0
    while data_received < n_int: #for the number of sample points
        dat_str = ser.read_until(b'\n');  # get the data as a string, ints seperated by spaces
        dat_int = list(map(int,dat_str.split())) # now the data is a list
        ref.append(dat_int[0])
        data.append(dat_int[1])
        data_received = data_received + 1
    meanzip = zip(ref,data)
    meanlist = []
    for i,j in meanzip:
        meanlist.append(abs(i-j))
    score = mean(meanlist)
    t = [i / 200 for i in range(len(ref))]  # time array in seconds
    #t = range(len(ref)) # index array
    plt.plot(t,ref,'r*-',t,data,'b*-')
    plt.title('Score = ' + str(score))
    plt.ylabel('Current (mA)')
    plt.xlabel('Time (s)')
    plt.show()

def read_plot_matrix2(): #position test (floats)
    n_str = ser.read_until(b'\n');  # get the number of data points to receive
    n_int = int(n_str) # turn it into an int
    print('Data length = ' + str(n_int))
    ref = []
    data = []
    data_received = 0
    while data_received < n_int:
        dat_str = ser.read_until(b'\n');  # get the data as a string, ints seperated by spaces
        dat_f = list(map(float,dat_str.split())) # now the data is a list
        ref.append(dat_f[0])
        data.append(dat_f[1])
        data_received = data_received + 1
    meanzip = zip(ref,data)
    meanlist = []
    for i,j in meanzip:
        meanlist.append(abs(i-j))
    score = mean(meanlist)
    t = [i / 200 for i in range(len(ref))]  # time array in seconds
    #t = range(len(ref)) # index array
    plt.plot(t,ref,'r*-',t,data,'b*-')
    plt.title('Score = ' + str(score))
    plt.ylabel('Motor Angle (degree)')
    plt.xlabel('Time (s)')
    plt.show() 


has_quit = False
# menu loop
while not has_quit:
    print('PIC32 MOTOR DRIVER INTERFACE\n')
    # display the menu options; this list will grow
    print('b: Read Current (mA) \t\tc: Get Encoder Count \td: Get Encoder Count (degrees) \te: Reset Encoder Count \
           \nf: Set PWM (-100 to 100) \tg: Set Current Gains \th: Get Current Gains \t\ti: Set Position Gains \
          \nj: Get Position Gains \t\tk: Test Current Gains \tl: Go to Angle (degrees) \tm: Load Step Trajectory \
          \nn: Load Cubic Trajectory \to: Execute Trajectory \tp: Unpower Motor \t\tq: Quit \
          \nr: Get Mode')
    # read the user's choice
    selection = input('\nENTER COMMAND: ')
    selection_endline = selection+'\n'
     
    # send the command to the PIC32
    ser.write(selection_endline.encode()); # .encode() turns the string into a char array
    
    # take the appropriate action
    # there is no switch() in python, using if elif instead
    if (selection == 'd'): #degree count
        b_count = ser.read_until(b'\n') #pic sends encoder count
        f_count = float(b_count) #convert to float
        print('The motor is at encoder degree: ' + str(f_count) + '\n') #print encoder count to screen
    elif (selection == 'q'): #quit
        print('Exiting client')
        has_quit = True; # exit client
        # be sure to close the port
        ser.close()
    elif (selection == 'b'): #get current
        b_mA = ser.read_until(b'\n')
        mA = float(b_mA)
        print('Current through motor is at: ' + str(mA) + '\n')
    elif (selection == 'g'): #set current gains
        n1 = input('Enter Kp gain: ') # get the numbers to send
        n2 = input('Enter Ki gain: ') 
        n1_int = float(n1) # turn it into an int
        n2_int = float(n2) # turn it into an int
        print('Kp, Ki = ' + str(n1) + ', ' + str(n2) + '\n') # print it to the screen to double check
        ser.write((str(n1_int) + " " + str(n2_int) + '\n').encode()); # send the gains
    elif (selection == 'i'): #set posn gains
        n1 = input('Enter Kp gain: ') # get the numbers to send
        n2 = input('Enter Ki gain: ') 
        n3 = input('Enter Kd gain: ') 
        n1_int = float(n1) # turn it into an int
        n2_int = float(n2) # turn it into an int
        n3_int = float(n3) # turn it into an int
        print('Kp, Ki, Kd = ' + str(n1) + ', ' + str(n2) + ', ' + str(n3) + '\n') # print it to the screen to double check
        ser.write((str(n1_int) + " " + str(n2_int) + " " + str(n3_int) + '\n').encode()); # send the gains
    elif (selection == 'h'): #get current gains
        b_gains = ser.read_until(b'\n') 
        print(b_gains.decode('utf-8').rstrip('\r\n') +'\n') #print gains to screen
    elif (selection == 'l'): #go to posn
        degree = input('Enter number: ') # get the number to send
        ser.write((str(degree)+'\n').encode()) #send the number
    elif (selection == 'j'): #get posn gains
        b_gains = ser.read_until(b'\n') 
        print(b_gains.decode('utf-8').rstrip('\r\n') +'\n') #print gains to screen 
    elif (selection == 'e'): #reset count
        print('The motor encoder count has been reset to 0.\n')
    elif (selection == 'c'): #motor count
        b_count = ser.read_until(b'\n')
        int_count = int(b_count)
        print('The motor is at encoder count: ' + str(int_count) + '\n') #print encoder count to screen
    elif (selection == 'k'): #test curr gains
        read_plot_matrix()
    elif (selection == 'r'): #get the mode from pic
        b_mode = ser.read_until(b'\n') 
        print('The motor is currently in: ' + b_mode.decode('utf-8').rstrip('\r\n') +'\n') #print mode to screen 
        #read until returns the txt from the pic in byte mode. This is then decoded into typ utf-8 and stripped of \n, \r to clean up    
    elif (selection == 'n'): #plot cubic trajectory
        ref = genRef('cubic')
        print(len(ref)) #print the num of samples in the reference trajectory 
        t = range(len(ref))
        plt.plot(t,ref,'r*-')
        plt.ylabel('Motor Angle (degrees)')
        plt.xlabel('Sample Number')
        plt.show()
        # send 
        ser.write((str(len(ref)) + '\n').encode()) #send length to pic32
        for i in ref:
            ser.write((str(i)+'\n').encode())  # Send each ref data point       
    elif (selection == 'm'): #plot step trajectory
        ref = genRef('step')
        print(len(ref))
        t = range(len(ref))
        plt.plot(t,ref,'r*-')
        plt.ylabel('Motor Angle (degrees)')
        plt.xlabel('Sample Number')
        plt.show()
        # send 
        ser.write((str(len(ref))+'\n').encode())
        for i in ref:
            ser.write((str(i)+'\n').encode())
    elif (selection == 'o'):
        read_plot_matrix2()
    else:
        print('Invalid Selection ' + selection_endline)


