"""
Script to read Inputs from Playstaion 5 DualSense Controller

The final idea is to use this controller to control the Robotic Arm
"""

import hid
import time

#make a class that contains all the controller button states
class Controller:
    def __init__(self):
        self.LXaxis = 0
        self.LYaxis = 0
        self.RXaxis = 0
        self.RYaxis = 0
        self.Dpad = 0
        self.Triangle = 0
        self.Circle = 0
        self.Cross = 0
        self.Square = 0
        self.L1 = 0
        self.L2 = 0
        self.R1 = 0
        self.R2 = 0

    def UpdateVariables(self,report):
        self.LXaxis = report[1]
        self.LYaxis = report[2]
        self.RXaxis = report[3]
        self.RYaxis = report[4]
        #For D pad, report[5]'s first 4 bits are used
        self.Dpad =  report[5] & 0x0F
        self.Triangle = (report[5] & 0x80) >> 7
        self.Circle = (report[5] & 0x40) >> 6 
        self.Cross = (report[5] & 0x20) >> 5
        self.Square = (report[5] & 0x10) >> 4
        self.L1 = report[6] & 0x01
        self.R1 = (report[6] & 0x02) >> 1 
        self.L2 = report[8]
        self.R2 = report[9]

    def SpitData(self):
        #output a array of all the variables
        return [self.LXaxis,self.LYaxis,self.RXaxis,self.RYaxis,self.Dpad,self.Triangle,self.Circle,self.Cross,self.Square,self.L1,self.L2,self.R1,self.R2]

    def PrintData(self):
        print("LXaxis: ",self.LXaxis)
        print("LYaxis: ",self.LYaxis)
        print("RXaxis: ",self.RXaxis)
        print("RYaxis: ",self.RYaxis)
        print("Dpad: ",self.Dpad)
        print("Triangle: ",self.Triangle)
        print("Circle: ",self.Circle)
        print("Cross: ",self.Cross)
        print("Square: ",self.Square)
        print("L1: ",self.L1)
        print("L2: ",self.L2)
        print("R1: ",self.R1)
        print("R2: ",self.R2)
    

for device in hid.enumerate():
    print(f"0x{device['vendor_id']:04x}:0x{device['product_id']:04x} {device['product_string']}")

# 054c:0ce6 Sony Corp. Wireless Controller
gamepad = hid.device()
gamepad.open(0x054c, 0x0ce6)
gamepad.set_nonblocking(True)

#Instantiate a controller object
controller = Controller()

while True:
    report = gamepad.read(64)
    if report:
        #print only first 10 bytes
        # report = report[:10]
        # print(report)
        # wait for 1 second
        #call a function to fill the variables
        controller.UpdateVariables(report)
        print("Data Parsed:")
        controller.PrintData()
        time.sleep(0.1)
        #clear the screen
        print(chr(27) + "[2J")
        #move cursor to top left
        print(chr(27) + "[0;0H")
    
