/*
    * Controller.cpp
    *
    *  Created on: 06-05-2023
    *  Author: Nischay Joshi
    * Description: This File hosts all the functions for the Controller input

*/

#include "Controller.h"

int Controller::GetControllerData(){
    //If Serial Data is not available
    if(!Serial.available()){
        return 0;
    }
    //Read the Serial Data
    while(Serial.available()){
        String SerialData = Serial.readStringUntil('\n');
        //Now check if the Data's first item is the header
        if(SerialData.substring(0, SerialData.indexOf(',')) == this->Header){
            //Now we need to get the data
            for(int i = 0; i < ControllerDataLength; i++){
                SerialData = SerialData.substring(SerialData.indexOf(',') + 1);
                this->Data[i] = SerialData.substring(0, SerialData.indexOf(',')).toInt();
            }
        }
    }
    return 1;
}

void Controller::PrintControllerData(){
    Serial.print("Controller Data: ");
    for(int i = 0; i < ControllerDataLength; i++){
        Serial.print(this->Data[i]);
        Serial.print(", ");
    }
    Serial.println();
}