/*
    * ServoLib.cpp
    *
    * Created: 05-04-2023
    * Author: Nischay Joshi
    * Description: This is a library for controlling a servo motor
*/

#include "ServoLib.h"

//Add the functions from the header file here

/*
    ServoLib Constructor
    @param servoPin: The pin to which the servo is connected
    @param MaxPos: The maximum position of the servo
    @param MinPos: The minimum position of the servo
    @param Speed: The speed at which the servo should move
*/
ServoLib::ServoLib(uint8_t servoPin, int MaxPos, int MinPos, uint16_t Speed, int Default){
    this->servoPin = servoPin;
    this->MaxPos = MaxPos;
    this->MinPos = MinPos;
    this->Speed = Speed;
    //this->DefaultPos = MinPos + (MaxPos - MinPos)/2;
    this->DefaultPos = Default;
    //Serial.print("Default Pos set to: ");
    //Serial.println(this->DefaultPos);
    this->SetDelay();
}

/*
    ServoGoto: This function moves the servo to the desired position
    @param Pos: The position to which the servo should move
    The Function makes sure that the servo does not move beyond the maximum and minimum positions
*/
void ServoLib::ServoGoto(int Pos){

    if(this->CurrentPos == Pos || Pos > this->MaxPos || Pos < this->MinPos){
        //sprintf(this->buffer, "Servo GotoPOS Escaped\n");
        //Serial.print(this->buffer);
        return;
    }
    else{        
        //Now based on the speed, move the servo to the desired position
        if(this->CurrentPos > Pos){
            for(int i = this->CurrentPos; i > Pos; i-=this->Increment){
                this->servo.write(i);
                //sprintf(this->buffer, "Servo Value written: %d\n", i);
                //Serial.print(this->buffer);
                delay(this->Delay);
            }
        }
        else{
            for(int i = this->CurrentPos; i < Pos; i+=this->Increment){
                this->servo.write(i);
                //sprintf(this->buffer, "Servo Value written: %d\n", i);
                //Serial.print(this->buffer);
                delay(this->Delay);
            }
        }
        //Update the current position
        this->servo.write(Pos);
        this->CurrentPos = Pos; 
    }
}


/*
    ServoSetPos: This function sets the position of the servo
    @param Pos: The position to which the servo should move
    The Function makes sure that the servo does not move beyond the maximum and minimum positions
*/
void ServoLib::ServoSetPos(int Pos){
    if(Pos > this->MaxPos || Pos < this->MinPos){
        //sprintf(this->buffer, "Servo SetPOS Escaped\n");
        //Serial.print(this->buffer);
        return;
    }
    else{
        this->servo.write(Pos);
        this->CurrentPos = Pos;
    }
}

/*
    SetDelay: This function sets the delay based on the speed
*/
void ServoLib::SetDelay(){
    //Case Statement
    switch(this->Speed){
        case 1: 
            this->Delay = 22;
            this->Increment = 1;
            break;
        case 2:
            this->Delay = 15;
            this->Increment = 1;
            break;
        case 3:
            this->Delay = 8;
            this->Increment = 1;
            break;
        case 4:
            this->Delay = 4;
            this->Increment = 1;
            break;
        case 5:
            this->Delay = 1;
            this->Increment = 1;
            break;
        default:
            this->Delay = 40;
            this->Increment = 1;
            break;
    }
}

/*
    UpdateServoParams: This function updates the parameters of the servo
    @param MaxPos: The maximum position of the servo
    @param MinPos: The minimum position of the servo
    @param Speed: The speed at which the servo should move
*/
void ServoLib::UpdateServoParams(int MaxPos, int MinPos, uint16_t Speed){
    this->MaxPos = MaxPos;
    this->MinPos = MinPos;
    this->Speed = Speed;
    this->SetDelay();
}

/*
    SetupServo: This function sets up the servo
*/
void ServoLib::SetupServo(){
    this->servo.attach(servoPin);
    this->servo.write(this->CurrentPos);
    //Put the Servo to the initial position
    this->ServoGoto(this->CurrentPos);
}