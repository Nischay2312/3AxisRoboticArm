/*
    * Constants.h
    *
    * Created on: 05/04/2023
    * This Header files contains all the constants used in the 3AxisRobotic Arm project 
*/

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include <Arduino.h>

// System Constants
#define SYSTEM_VERSION "1.0.0"
#define SYSTEM_NAME "3AxisRoboticArm"
#define SYSTEM_AUTHOR "Nischay Joshi"
#define LOOP_FREQUENCY 50 
#define LOOP_DELAY 1000/LOOP_FREQUENCY

// Constants for Analog input pins
#define POT_PIN A0
#define POT_MIN 0
#define POT_MAX 1023


// Base Servo Constants
#define BASE_SERVO_PIN 9
#define BASE_SERVO_MIN 0
#define BASE_SERVO_MAX 180
#define BASE_SERVO_SPEED 4

// Function Declarations
void GreetUser();

#endif