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

void setup() {
  Serial.begin(9600);
  BaseServo.SetupServo();
  GreetUser();
  pinMode(POT_PIN, INPUT);
}

int BasePos = 0;
bool ReadPot = 0;
 
void loop() {

  while(Serial.available() > 0){
    String SerialData = Serial.readStringUntil('\n');
    //if string is exactly like this: "flip", then the base arm will take in potentiometer input
    if(SerialData == "flip"){
        ReadPot = !ReadPot;
        if(ReadPot){
          BaseServo.UpdateServoParams(BASE_SERVO_MAX, BASE_SERVO_MIN, 5);
        }
        else{
          BaseServo.UpdateServoParams(BASE_SERVO_MAX, BASE_SERVO_MIN, BASE_SERVO_SPEED);
        }
      }
    Serial.print("Serial Data Echoed: ");
    Serial.println(SerialData);
    if(!ReadPot){
      BasePos = SerialData.substring(0, SerialData.indexOf(',')).toInt();
      SerialData = SerialData.substring(SerialData.indexOf(',') + 1);
      int ShoulderPos = SerialData.substring(0, SerialData.indexOf(',')).toInt();
      SerialData = SerialData.substring(SerialData.indexOf(',') + 1);
      int ElbowPos = SerialData.toInt();
      Serial.println("BasePos: " + String(BasePos) + " ShoulderPos: " + String(ShoulderPos) + " ElbowPos: " + String(ElbowPos));  
      BaseServo.ServoGoto(BasePos);
    }
  } 
  if(ReadPot){
    int value = analogRead(POT_PIN);
    BasePos = map(value, 250, 1023, BASE_SERVO_MIN, BASE_SERVO_MAX);
    BaseServo.ServoGoto(BasePos);
    // Serial.print("Potentiometer Value: ");
    // Serial.println(value);
    // Serial.println("BasePos: " + String(BasePos));
    // delay(500);  
  }
  delay(LOOP_DELAY);
}
