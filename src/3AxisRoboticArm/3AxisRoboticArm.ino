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
#define StickDeadZone 20
#define BaseTurnSpeed 0.04
#define LowArmTurnSpeed 0.02
#define UpArmTurnSpeed 0.015

//Global Variables
float BaseAngle;
float LowArmAngle;
float UpArmAngle;
bool GripperState;  //0 = Close, 1 = Open

void ECHOSerialData();

void setup() {
  Serial.begin(115200);
  //Dont continue until Serial Comminication is established
  while(!Serial);

  //Setup the Servos
  BaseServo.SetupServo();
  LowArmServo.SetupServo();
  UpArmServo.SetupServo();
  GripperServo.SetupServo();
  
  //Intialize the Servo Angles
  BaseAngle = BaseServo.DefaultPos;
  LowArmAngle = LowArmServo.DefaultPos;
  UpArmAngle = UpArmServo.DefaultPos;
  GripperState = 0;

  //Here we will make sure that the RobotArmInput is setup and ready to go
  if(!RobotArmInput.SetupController()){
    Serial.println("Controller Setup Failed");
    while(1);
  }
  RobotArmInput.UpdateDataAverage();
  Serial.println("Setup Complete");
  delay(1000);  
}
 
void loop() {
  //Get the Controller Data  
  if(RobotArmInput.GetControllerData()){
    //Print the Controller Data
    RobotArmInput.PrintControllerData();
    
    // //Set the Servo Angles
    // int BaseAngle = map(RobotArmInput.Data[0], 0, 255, BASE_SERVO_MAX, BASE_SERVO_MIN);
    // int LowArmAngle = map(RobotArmInput.Data[1], 0, 255, LOWARM_SERVO_MAX, LOWARM_SERVO_MIN);
    // int UpArmInput1 = map(RobotArmInput.Data[2], -255, 255, UPARM_SERVO_MAX, UPARM_SERVO_MIN);
    // int UpArmInput2 = map(-1*RobotArmInput.Data[3], -255, 255, UPARM_SERVO_MAX, UPARM_SERVO_MIN);
    // int UpArmAngle = RobotArmInput.Data[2] > 0 ? UpArmInput1 : UpArmInput2;
    
    //Set the Servo Angles
    BaseAngle +=  abs(RobotArmInput.DataAverage[0] - RobotArmInput.Data[0]) > StickDeadZone? ((RobotArmInput.DataAverage[0] - RobotArmInput.Data[0])*BaseTurnSpeed) : 0;
    LowArmAngle +=  abs(RobotArmInput.DataAverage[1] - RobotArmInput.Data[1]) > StickDeadZone? ((RobotArmInput.DataAverage[1] - RobotArmInput.Data[1])*LowArmTurnSpeed) : 0;
    int UpArm1 = RobotArmInput.Data[2] - RobotArmInput.DataAverage[2];
    int UpArm2 = RobotArmInput.Data[3] - RobotArmInput.DataAverage[3];
    UpArmAngle += abs(UpArm1) > StickDeadZone ? (UpArm1 * UpArmTurnSpeed) : 0; 
    UpArmAngle -= abs(UpArm2) > StickDeadZone ? (UpArm2 * UpArmTurnSpeed) : 0;

    //Check if the Angles are within the limits
    UpArmAngle = UpArmAngle < 38 + 40 + UPARM_SERVO_MIN - LowArmAngle ? 38 + 40 + UPARM_SERVO_MIN - LowArmAngle  : UpArmAngle;
    UpArmAngle = UpArmAngle > 159 + 40 + UPARM_SERVO_MAX - LowArmAngle ? 159 + 40 + UPARM_SERVO_MAX - LowArmAngle  : UpArmAngle;
    BaseAngle = BaseAngle > BASE_SERVO_MAX ? BASE_SERVO_MAX : BaseAngle;
    BaseAngle = BaseAngle < BASE_SERVO_MIN ? BASE_SERVO_MIN : BaseAngle;
    LowArmAngle = LowArmAngle > LOWARM_SERVO_MAX ? LOWARM_SERVO_MAX : LowArmAngle;
    LowArmAngle = LowArmAngle < LOWARM_SERVO_MIN ? LOWARM_SERVO_MIN : LowArmAngle;
    UpArmAngle = UpArmAngle > UPARM_SERVO_MAX ? UPARM_SERVO_MAX : UpArmAngle;
    UpArmAngle = UpArmAngle < UPARM_SERVO_MIN ? UPARM_SERVO_MIN : UpArmAngle;

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

    //If Reset is pressed, reset the servos to their default positions
    if(RobotArmInput.Data[6]){
      BaseAngle = BaseServo.DefaultPos;
    }
    if(RobotArmInput.Data[7]){
      LowArmAngle = LowArmServo.DefaultPos;
    }
    if(RobotArmInput.Data[8]){
      UpArmAngle = UpArmServo.DefaultPos;
    }
    //Send the Servo Angles
   BaseServo.ServoSetPos((int)BaseAngle);
   LowArmServo.ServoSetPos((int)LowArmAngle);
   UpArmServo.ServoSetPos((int)UpArmAngle);
  }

  //Debug Statements
  // Serial.print("Base Angle Received: ");
  // Serial.println(BaseAngle);
  // Serial.print("Low Arm Angle Received: ");
  // Serial.println(LowArmAngle);
  // Serial.print("Up Arm Angle Received: ");
  // Serial.println(UpArmAngle);

  delay(LOOP_DELAY);
}

void ECHOSerialData(){
  if(Serial.available() > 0){
    Serial.write(Serial.read());
  } 
}