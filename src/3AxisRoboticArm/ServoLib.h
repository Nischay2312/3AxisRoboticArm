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

class ServoLib{
    private:
        uint8_t servoPin;
        int16_t CurrentPos = 0;
        int16_t MaxPos;
        int16_t MinPos;
        uint16_t Speed;
        uint16_t Delay;
        Servo servo;
        void SetDelay();

    public:
        ServoLib(uint8_t servoPin, int16_t MaxPos, int16_t MinPos, uint16_t Speed);
        void ServoGoto(int16_t Pos);
        void UpdateServoParams(int16_t MaxPos, int16_t MinPos, uint16_t Speed);
};

#endif