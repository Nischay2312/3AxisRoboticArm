"""
    helperfunctions.py

    Author: Nischay Joshi
    Date: 06-05-2023

    This file contains helper functions that can be used by other scripts.

"""

"""
Function to clear the screen. 
On calling will clear the terminal screen 
"""
def ClearScreen():
    print(chr(27) + "[2J")
    print(chr(27) + "[0;0H")
