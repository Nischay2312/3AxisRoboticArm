/*
    * Constants.cpp
    *
    * Created on: 05/04/2023
    * Author: Nischay Joshi
    * 
    * This file contains the implementation of the functions declared in the Constants.h file
*/

#include "Constants.h"

void GreetUser(){
  //Greet the user
  Serial.print("Welcome to ");
  Serial.println(SYSTEM_NAME);
  Serial.print("Version: ");
  Serial.println(SYSTEM_VERSION);
  Serial.print("Author: ");
  Serial.println(SYSTEM_AUTHOR);
  Serial.println("Serial Data Format: <BasePos>, <ShoulderPos>, <ElbowPos>");
  Serial.println("--------------------------------------------");
}