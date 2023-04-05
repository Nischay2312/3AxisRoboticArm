/*
  3 Axis Robotic Arm
  by: Nischay Joshi
  Date: 05/04/2023
  Description: This is a 3 axis robotic arm. Currently controlled by Serial Input
*/

#include "ServoLib.h"
#include "Constants.h"

//Create the base servo Object
ServoLib BaseServo(BASE_SERVO_PIN, BASE_SERVO_MAX, BASE_SERVO_MIN, BASE_SERVO_SPEED);

//Function Declarations
void GreetUser();

void setup() {
  Serial.begin(9600);
  BaseServo.SetupServo();
  GreetUser();
}

int BasePos = 0;
 
void loop() {
  //Read Serial Data, The format is as follows:
  //Serial Data Format: <BasePos>, <ShoulderPos>, <ElbowPos>
  //Example: 90, 90, 90
  while(Serial.available() > 0){
    String SerialData = Serial.readStringUntil('\n');
    Serial.print("Serial Data Echoed: ");
    Serial.println(SerialData);
    BasePos = SerialData.substring(0, SerialData.indexOf(',')).toInt();
    SerialData = SerialData.substring(SerialData.indexOf(',') + 1);
    int ShoulderPos = SerialData.substring(0, SerialData.indexOf(',')).toInt();
    SerialData = SerialData.substring(SerialData.indexOf(',') + 1);
    int ElbowPos = SerialData.toInt();
    Serial.println("BasePos: " + String(BasePos) + " ShoulderPos: " + String(ShoulderPos) + " ElbowPos: " + String(ElbowPos));  
    BaseServo.ServoGoto(BasePos);
  }  
  delay(LOOP_DELAY);
}

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
