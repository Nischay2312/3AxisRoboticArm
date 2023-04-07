"""
Script to read Inputs from Playstaion 5 DualSense Controller

The final idea is to use this controller to control the Robotic Arm
"""

import hid
import time
import controller
import helperfunctions


for device in hid.enumerate():
    print(f"0x{device['vendor_id']:04x}:0x{device['product_id']:04x} {device['product_string']}")

#Instantiate a controller object
PS5Controller = controller.Controller()

while True:
    PS5Controller.ReadController()
    print("Data Parsed:")
    PS5Controller.PrintData()
    print(PS5Controller.GetControllerState())
    time.sleep(0.1)
    #clear the screen
    helperfunctions.ClearScreen()