/*
    * ServoLib.h
    *
    * Created: 05/04/2023
    * Author:  Nischay Joshi
    * Servo Library for 3 Axis Robotic Arm. Uses the exisitong Servo Library. Adds some extra functionality. 
*/

#ifndef SERVOLIB_H_
#define SERVOLIB_H_

#include <Servo.h>
#include "Arduino.h"
#include <stdio.h>
#include <stdlib.h>

class ServoLib{
    private:
        uint8_t servoPin;
        int CurrentPos = 0;
        int MaxPos;
        int MinPos;
        uint16_t Speed;
        uint8_t Increment = 1;
        uint16_t Delay;
        char buffer[500];
        Servo servo;
        void SetDelay();

    public:
        int DefaultPos;
        ServoLib(uint8_t servoPin, int MaxPos, int MinPos, uint16_t Speed, int Default);
        void ServoSetPos(int Pos);
        void ServoGoto(int Pos);
        void UpdateServoParams(int MaxPos, int MinPos, uint16_t Speed);
        void SetupServo();
};

#endif