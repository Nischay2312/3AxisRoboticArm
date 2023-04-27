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

RUNRATE = 50  #Hz
DEBUG = True
DEBUGCONTROLLER = False

# serial port settings
port = "COM13"
baudrate = 115200
bytesize = serial.EIGHTBITS
parity = serial.PARITY_NONE
stopbits = serial.STOPBITS_ONE

# open the serial port
RobotArmSerial = serial.Serial(port, baudrate, bytesize, parity, stopbits, timeout=0.1)
time.sleep(2)
print("Serial port", port, "opened")

InputsWanted = ["LXaxis","LYaxis", "L2", "R2", "L1", "R1", "Triangle", "Circle", "Square", "RXaxis", "RYaxis"]
DataToSend = {"Header": "FF"}
OutputString = ""

#Instantiate a controller object
PS5Controller = Controller()

#read the Controller inputs 
ControllerState = PS5Controller.GetControllerState()

#convert the dictionaly values to string but only for the inputs we want
for key in InputsWanted:
    if key in ControllerState:
        DataToSend[key] = str(ControllerState[key])

#now prepare the output String
for key in DataToSend:
    OutputString += DataToSend[key] + ","
OutputString = OutputString[:-1] + "\n"

#Wait untill we get an "FF" from the Robot Arm
while True:
    if RobotArmSerial.in_waiting > 0:
        DataFromRobotArm = RobotArmSerial.readline().decode().strip()
        if DataFromRobotArm == "FF":
            print("Robot Arm Ready")
            #send the PreRead data
            RobotArmSerial.write(bytes(OutputString, 'utf-8'))
            break

#The connection is established, now periodically send the contorller data to the robot arm
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


