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
#define SYSTEM_NAME "5AxisRoboticArm"
#define SYSTEM_AUTHOR "Nischay Joshi"
#define LOOP_FREQUENCY 100 
#define LOOP_DELAY 1000/LOOP_FREQUENCY

// Base Servo Constants
#define BASE_SERVO_PIN 9
#define BASE_SERVO_MIN 0
#define BASE_SERVO_MAX 180
#define BASE_SERVO_DEF 90
#define BASE_SERVO_SPEED 3

// Lower Arm Servo Constants
#define LOWARM_SERVO_PIN 10
#define LOWARM_SERVO_MIN 0
#define LOWARM_SERVO_MAX 180
#define LOWARM_SERVO_DEF 20
#define LOWARM_SERVO_SPEED 2

// Upper Arm Servo Constants
#define UPARM_SERVO_PIN 11
#define UPARM_SERVO_MIN 0
#define UPARM_SERVO_MAX 180
#define UPARM_SERVO_DEF 170
#define UPARM_SERVO_SPEED 2

//Wrist Roll Servo Constants
#define WRIST_R_SERVO_PIN 8
#define WRIST_R_SERVO_MAX 180
#define WRIST_R_SERVO_MIN 10
#define WRIST_R_SERVO_DEF 85
#define WRIST_R_SERVO_SPEED 2

//Wrist Pitch Servo Constants
#define WRIST_P_SERVO_PIN 7
#define WRIST_P_SERVO_MAX 180
#define WRIST_P_SERVO_MIN 0
#define WRIST_P_SERVO_DEF 90
#define WRIST_P_SERVO_SPEED 2

//Gripper Servo Constants
#define GRIPPER_SERVO_PIN 12
#define GRIPPER_SERVO_MIN 0
#define GRIPPER_SERVO_CLOSE 105
#define GRIPPER_SERVO_OPEN 170
#define GRIPPER_SERVO_MAX 180
#define GRIPPER_SERVO_SPEED 5

// Function Declarations
void GreetUser();

#endif