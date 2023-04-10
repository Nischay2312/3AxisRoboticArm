"""
EyeController.py

Author: Nischay Joshi
Date: 10-04-2023

This script is used to control the Eyes On he LCD Display

"""

from controller import Controller
import helperfunctions
import time
import serial

RUNRATE = 5  #Hz
DEBUG = True
DEBUGCONTROLLER = False

# serial port settings
port = "COM20"
baudrate = 115200
bytesize = serial.EIGHTBITS
parity = serial.PARITY_NONE
stopbits = serial.STOPBITS_ONE

# open the serial port
LCDDisplaySerial = serial.Serial(port, baudrate, bytesize, parity, stopbits, timeout=0.1)
time.sleep(2)
print("Serial port", port, "opened")

InputsWanted = ["LXaxis","LYaxis"]
DataToSend = {"Header": "FF"}
OutputString = ""

#Instantiate a controller object
PS5Controller = Controller()

#Wait till we read a "Setup Routine Done" from the LCD Display
# while True:
#     if LCDDisplaySerial.in_waiting > 0:
#         DataFromLCD = LCDDisplaySerial.readline().decode().strip()
#         if DataFromLCD == "Setup Routine Done":
#             print("LCD Display Ready")
#             break

#The connection is established, now periodically send the contorller data to the robot arm
while True:
    #quit the program on Keyboad Interrupt
    try:
        #Check if we read something from the Robot Arm
        if LCDDisplaySerial.in_waiting > 0:
            while LCDDisplaySerial.in_waiting > 0:
                #Read the data
                DataFromRobotArm = LCDDisplaySerial.readline().decode().strip()
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
        LCDDisplaySerial.write(bytes(OutputString, 'utf-8'))

        # Clear the output string
        OutputString = ""
        time.sleep(1/RUNRATE)

        #clear the screen
        helperfunctions.ClearScreen()

    except KeyboardInterrupt:
        print("Keyboard Interrupt, Program Terminated")
        break


