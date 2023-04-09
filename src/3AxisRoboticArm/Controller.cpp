/*
    * Controller.cpp
    *
    *  Created on: 06-05-2023
    *  Author: Nischay Joshi
    * Description: This File hosts all the functions for the Controller input

*/

#include "Controller.h"

/*
    * GetControllerData()
    * This function gets the data from the controller and stores it in the Data array
    * Returns 1 if data is available and 0 if no data is available
*/
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

/*
    * PrintControllerData()
    * This function prints the data from the Data array
*/
void Controller::PrintControllerData(){
    Serial.print("Controller Data: ");
    for(int i = 0; i < ControllerDataLength; i++){
        Serial.print(this->Data[i]);
        Serial.print(", ");
    }
    Serial.println();
}

/*
    * UpdateDataAverage()
    * This function updates the DataAverage array by taking the average of the last 20 values
*/
void Controller::UpdateDataAverage(){
    int interations = 20;
    //Wait until the Serial Data is available
    while(!this->GetControllerData());
    for(int i = 0; i < interations; i++){
        this->GetControllerData();
        for(int j = 0; j < ControllerDataLength; j++){
            this->DataAverage[j] += this->Data[j];
        }        
    }
    for(int i = 0; i < ControllerDataLength; i++){
        this->DataAverage[i] /= interations;
    }
}

/*
    * SetupController()
    * This function sets up the controller and returns 1 if successful and 0 if not
*/
int Controller::SetupController(){
    //The plan is to send a command to the controller and wait for a response
    //Now we wait for the response, the controller will start outputting the data
    //We will wait for 1000 iterations
    int timeout = 1000;
    while(timeout > 0){
        //Send the command
        Serial.println("FF");
        delay(10);
        if(this->GetControllerData()){
            return 1;
        }
        timeout--;
    }
    return 0;
}