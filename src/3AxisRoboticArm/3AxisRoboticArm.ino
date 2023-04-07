/*
  3 Axis Robotic Arm
  by: Nischay Joshi
  Date: 05/04/2023
  Description: This is a 3 axis robotic arm. Currently controlled by Serial Input
*/

#include "ServoLib.h"
#include "Constants.h"
#include "Controller.h"

//Create the base servo Object
ServoLib BaseServo(BASE_SERVO_PIN, BASE_SERVO_MAX, BASE_SERVO_MIN, BASE_SERVO_SPEED);
ServoLib LowArmServo(LOWARM_SERVO_PIN, LOWARM_SERVO_MAX, LOWARM_SERVO_MIN, LOWARM_SERVO_SPEED);
ServoLib UpArmServo(UPARM_SERVO_PIN, UPARM_SERVO_MAX, UPARM_SERVO_MIN, UPARM_SERVO_SPEED);
ServoLib GripperServo(GRIPPER_SERVO_PIN, GRIPPER_SERVO_MAX, GRIPPER_SERVO_MIN, GRIPPER_SERVO_SPEED);

//Create the Controller Object
Controller RobotArmInput;

//Global Variables
int BaseAngle = BaseServo.DefaultPos;
int LowArmAngle = LowArmServo.DefaultPos;
int UpArmAngle = UpArmServo.DefaultPos;
bool GripperState = 0;  //0 = Close, 1 = Open

void ECHOSerialData();

void setup() {
  Serial.begin(115200);
  BaseServo.SetupServo();
  LowArmServo.SetupServo();
  UpArmServo.SetupServo();
  GripperServo.SetupServo();
  GreetUser();
  pinMode(POT_PIN, INPUT);
}
 
void loop() {
  //Get the Controller Data  
  if(RobotArmInput.GetControllerData()){
    //Print the Controller Data
    RobotArmInput.PrintControllerData();
    
    // //Set the Servo Angles
    int BaseAngle = map(RobotArmInput.Data[0], 0, 255, BASE_SERVO_MAX, BASE_SERVO_MIN);
    int LowArmAngle = map(RobotArmInput.Data[1], 0, 255, LOWARM_SERVO_MAX, LOWARM_SERVO_MIN);
    int UpArmInput1 = map(RobotArmInput.Data[2], -255, 255, UPARM_SERVO_MAX, UPARM_SERVO_MIN);
    int UpArmInput2 = map(-1*RobotArmInput.Data[3], -255, 255, UPARM_SERVO_MAX, UPARM_SERVO_MIN);
    int UpArmAngle = RobotArmInput.Data[2] > 0 ? UpArmInput1 : UpArmInput2;
    
    //Set the Servo Angles
    //BaseAngle +=  RobotArmInput.Data[0];

    if(RobotArmInput.Data[4] == 1 && GripperState == 1){
      GripperState = 0;
      GripperServo.ServoGoto(GRIPPER_SERVO_CLOSE);
      Serial.println("Gripper Closed");
    }
    else if(RobotArmInput.Data[5] == 1 && GripperState == 0){
      GripperState = 1;
      GripperServo.ServoGoto(GRIPPER_SERVO_OPEN);
      Serial.println("Gripper Opened");
    }

    //Debug Statements
    Serial.print("Base Angle Received: ");
    Serial.println(BaseAngle);
    //Serial.print("Low Arm Angle Received: ");
    //Serial.println(LowArmAngle);
    //Serial.print("Up Arm Angle Received: ");
    //Serial.println(UpArmAngle);
    //Serial.print("Low Arm Angle Received: ");
    //Serial.println(LowArmAngle);
    //BaseServo.ServoGoto(BaseAngle);
    //LowArmServo.ServoGoto(LowArmAngle);
    BaseServo.ServoGoto(BaseAngle);
    LowArmServo.ServoGoto(LowArmAngle);
    UpArmServo.ServoGoto(UpArmAngle);
  }
  delay(LOOP_DELAY);
}

void ECHOSerialData(){
  if(Serial.available() > 0){
    Serial.write(Serial.read());
  } 
}