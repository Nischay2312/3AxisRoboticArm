"""
RobotArmControl.py

Author: Nischay Joshi
Date: 06-05-2023

This script is used to control the robot arm with the help of PS5 controller

"""

from controller import Controller
import helperfunctions
import time
import serial

RUNRATE = 25  #Hz
DEBUG = True
DEBUGCONTROLLER = False

#Instantiate a controller object
PS5Controller = Controller()

# serial port settings
port = "COM13"
baudrate = 115200
bytesize = serial.EIGHTBITS
parity = serial.PARITY_NONE
stopbits = serial.STOPBITS_ONE

# open the serial port
RobotArmSerial = serial.Serial(port, baudrate, bytesize, parity, stopbits)
time.sleep(2)
print("Serial port", port, "opened")

InputsWanted = ["LXaxis","RYaxis","L2", "R2", "L1", "R1"]
DataToSend = {"Header": "FF"}
OutputString = ""

while True:
    #quit the program on Keyboad Interrupt
    try:
        #Check if we read something from the Robot Arm
        if RobotArmSerial.in_waiting > 0:
            while RobotArmSerial.in_waiting > 0:
                #Read the data
                DataFromRobotArm = RobotArmSerial.readline().decode().strip()
                #Print the data
                if DEBUG:
                    print("Data From Robot Arm: ", DataFromRobotArm)

        #Read the controller
        ControllerState = PS5Controller.GetControllerState()
        #Print the Controller Inputs
        if DEBUGCONTROLLER:
             print(ControllerState)
        #convert the dictionaly values to string but only for the inputs we want
        for key in InputsWanted:
            if key in ControllerState:
                DataToSend[key] = str(ControllerState[key])
    
        #now prepare the output String
        for key in DataToSend:
            OutputString += DataToSend[key] + ","
        OutputString = OutputString[:-1] + "\n"

        if DEBUG:
            print(OutputString)
        
        # Send the Controller Inputs to the Robot Arm
        RobotArmSerial.write(bytes(OutputString, 'utf-8'))

        # Clear the output string
        OutputString = ""
        time.sleep(1/RUNRATE)

        #clear the screen
        helperfunctions.ClearScreen()

    except KeyboardInterrupt:
        print("Keyboard Interrupt, Program Terminated")
        break


