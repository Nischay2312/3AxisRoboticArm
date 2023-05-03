/*
    * Controller.h
    *
    *  Created on: 06-05-2023
    *      Author: Nischay Joshi
    * This file is part of 3AxisRoboticArm. Holds the function prototypes for parsing controller inputs.
*/

#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "Arduino.h"

#define ControllerDataLength 12

class Controller{
    public:
    int DataAverage[ControllerDataLength] = {0};
    int Data[ControllerDataLength] = {0};
    String Header = "FF";
    int GetControllerData();
    void PrintControllerData();
    void UpdateDataAverage();
    int SetupController();
};
#endif