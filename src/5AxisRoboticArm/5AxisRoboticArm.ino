/*
  5 Axis Robotic Arm
  by: Nischay Joshi
  Date: 05/04/2023
  Description: This is a Controller for 5 axis robotic arm. Currently controlled by Serial Input
               The Arm is designed by How To Mechatronics link: https://howtomechatronics.com/tutorials/arduino/diy-arduino-robot-arm-with-smartphone-control/
*/

#include "ServoLib.h"
#include "Constants.h"
#include "Controller.h"

//Create the base servo Object
ServoLib BaseServo(BASE_SERVO_PIN, BASE_SERVO_MAX, BASE_SERVO_MIN, BASE_SERVO_SPEED, BASE_SERVO_DEF, BASE_SRVO_LOW_WIDTH, BASE_SRVO_HIGH_WIDTH);
ServoLib LowArmServo(LOWARM_SERVO_PIN, LOWARM_SERVO_MAX, LOWARM_SERVO_MIN, LOWARM_SERVO_SPEED, LOWARM_SERVO_DEF, LOWARM_SRVO_LOW_WIDTH, LOWARM_SRVO_HIGH_WIDTH);
ServoLib UpArmServo(UPARM_SERVO_PIN, UPARM_SERVO_MAX, UPARM_SERVO_MIN, UPARM_SERVO_SPEED, UPARM_SERVO_DEF, UPARM_SRVO_LOW_WIDTH, UPARM_SRVO_HIGH_WIDTH);
ServoLib WristPitchServo(WRIST_P_SERVO_PIN, WRIST_P_SERVO_MAX, WRIST_P_SERVO_MIN, WRIST_P_SERVO_SPEED, WRIST_P_SERVO_DEF, WRIST_P_SRVO_LOW_WIDTH, WRIST_P_SRVO_HIGH_WIDTH);
ServoLib WristRollServo(WRIST_R_SERVO_PIN, WRIST_R_SERVO_MAX, WRIST_R_SERVO_MIN, WRIST_R_SERVO_SPEED, WRIST_R_SERVO_DEF, WRIST_R_SRVO_LOW_WIDTH, WRIST_R_SRVO_HIGH_WIDTH);
ServoLib GripperServo(GRIPPER_SERVO_PIN, GRIPPER_SERVO_MAX, GRIPPER_SERVO_MIN, GRIPPER_SERVO_SPEED, GRIPPER_SERVO_OPEN, GRIPPER_SRVO_LOW_WIDTH, GRIPPER_SRVO_HIGH_WIDTH);


//Create the Controller Object
Controller RobotArmInput;
#define StickDeadZone 40
#define BaseTurnSpeed 0.05
#define LowArmTurnSpeed 0.02
#define UpArmTurnSpeed 0.02
#define WristPitchTurnSpeed 0.04
#define WristRollTurnSpeed 0.04

//Global Variables
float BaseAngle;
float LowArmAngle;
float UpArmAngle;
float WristPitchAngle;
float WristRollAngle;
bool GripperState;  //0 = Close, 1 = Open
bool isAtDefault = false;
unsigned long updatedTime = 0;
unsigned long inputTime = 0;
unsigned long beatTime = 0;
unsigned long gripperTime = 0;
void ECHOSerialData();
void HeartBeat();

void setup() {
  Serial.begin(115200);
  //Dont continue until Serial Comminication is established
  while(!Serial);

  //Setup the Servos
  BaseServo.SetupServo();
  LowArmServo.SetupServo();
  UpArmServo.SetupServo();
  WristPitchServo.SetupServo();
  WristRollServo.SetupServo();
  GripperServo.SetupServo();
  
  //Intialize the Servo Angles
  BaseAngle = BaseServo.DefaultPos;
  LowArmAngle = LowArmServo.DefaultPos;
  UpArmAngle = UpArmServo.DefaultPos;
  WristPitchAngle = WristPitchServo.DefaultPos;
  WristRollAngle = WristRollServo.DefaultPos;
  GripperState = 0;
  
  //Send the Servo Angles
  BaseServo.ServoSetPos((int)BaseAngle);
  LowArmServo.ServoSetPos((int)LowArmAngle);
  UpArmServo.ServoSetPos((int)UpArmAngle);
  WristPitchServo.ServoSetPos((int)WristPitchAngle);
  WristRollServo.ServoSetPos((int)WristRollAngle);
  GripperServo.ServoGoto(GRIPPER_SERVO_OPEN);
  isAtDefault = true;

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
    inputTime = millis();
    //Print the Controller Data
    RobotArmInput.PrintControllerData();    
    // if(millis() - updatedTime < 10000){
    //   RobotArmInput.UpdateDataAverage();
    //   updatedTime = millis();
    // }
    //Set the Servo Angles
    BaseAngle +=  abs(RobotArmInput.DataAverage[0] - RobotArmInput.Data[0]) > StickDeadZone? ((RobotArmInput.DataAverage[0] - RobotArmInput.Data[0])*BaseTurnSpeed) : 0;
    LowArmAngle +=  abs(RobotArmInput.DataAverage[1] - RobotArmInput.Data[1]) > StickDeadZone? ((RobotArmInput.DataAverage[1] - RobotArmInput.Data[1])*LowArmTurnSpeed) : 0;
    int UpArm1 = RobotArmInput.Data[2] - RobotArmInput.DataAverage[2];
    int UpArm2 = RobotArmInput.Data[3] - RobotArmInput.DataAverage[3];
    UpArmAngle += abs(UpArm1) > StickDeadZone ? (UpArm1 * UpArmTurnSpeed) : 0; 
    UpArmAngle -= abs(UpArm2) > StickDeadZone ? (UpArm2 * UpArmTurnSpeed) : 0;
    // WristPitchAngle +=  abs(RobotArmInput.DataAverage[10] - RobotArmInput.Data[10]) > StickDeadZone? ((RobotArmInput.DataAverage[10] - RobotArmInput.Data[10])*WristPitchTurnSpeed*-1) : 0;
    // WristRollAngle +=  abs(RobotArmInput.DataAverage[9] - RobotArmInput.Data[9]) > StickDeadZone? ((RobotArmInput.DataAverage[9] - RobotArmInput.Data[9])*WristRollTurnSpeed*-1) : 0;

    //Check if the Angles are within the limits
    //UpArmAngle = UpArmAngle < 38 + 40 + UPARM_SERVO_MIN - LowArmAngle ? 38 + 40 + UPARM_SERVO_MIN - LowArmAngle  : UpArmAngle;
    //UpArmAngle = UpArmAngle > 159 + 40 + UPARM_SERVO_MAX - LowArmAngle ? 159 + 40 + UPARM_SERVO_MAX - LowArmAngle  : UpArmAngle;
    BaseAngle = BaseAngle > BASE_SERVO_MAX ? BASE_SERVO_MAX : BaseAngle;
    BaseAngle = BaseAngle < BASE_SERVO_MIN ? BASE_SERVO_MIN : BaseAngle;
    LowArmAngle = LowArmAngle > LOWARM_SERVO_MAX ? LOWARM_SERVO_MAX : LowArmAngle;
    LowArmAngle = LowArmAngle < LOWARM_SERVO_MIN ? LOWARM_SERVO_MIN : LowArmAngle;
    UpArmAngle = UpArmAngle > UPARM_SERVO_MAX ? UPARM_SERVO_MAX : UpArmAngle;
    UpArmAngle = UpArmAngle < UPARM_SERVO_MIN ? UPARM_SERVO_MIN : UpArmAngle;
    WristPitchAngle = WristPitchAngle > WRIST_P_SERVO_MAX ? WRIST_P_SERVO_MAX : WristPitchAngle;
    WristPitchAngle = WristPitchAngle < WRIST_P_SERVO_MIN ? WRIST_P_SERVO_MIN : WristPitchAngle;
    WristRollAngle = WristRollAngle > WRIST_R_SERVO_MAX ? WRIST_R_SERVO_MAX : WristRollAngle;
    WristRollAngle = WristRollAngle < WRIST_R_SERVO_MIN ? WRIST_R_SERVO_MIN : WristRollAngle;

    if(RobotArmInput.Data[4] == 1 && GripperState == 1){
      GripperState = 0;
      if(millis() - gripperTime > 1000){
        gripperTime = millis();
        GripperServo.ServoGoto(GRIPPER_SERVO_CLOSE);
        Serial.println("Gripper Closed");
      }
    }
    else if(RobotArmInput.Data[4] == 1 && GripperState == 0){
      GripperState = 1;
      if(millis() - gripperTime > 1000){
        gripperTime = millis();
        GripperServo.ServoGoto(GRIPPER_SERVO_OPEN);
        Serial.println("Gripper Opened");
      }
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
   WristPitchServo.ServoSetPos((int)WristPitchAngle);
   WristRollServo.ServoSetPos((int)WristRollAngle);
   isAtDefault = false;
  }

  //Debug Statements
  // Serial.print("Base Angle Received: ");
  // Serial.println(BaseAngle);
  // Serial.print("Low Arm Angle Received: ");
  // Serial.println(LowArmAngle);
  // Serial.print("Up Arm Angle Received: ");
  // Serial.println(UpArmAngle);
  if(millis() - inputTime > 1000){
    HeartBeat();
  }
  delay(LOOP_DELAY);
}

void ECHOSerialData(){
  if(Serial.available() > 0){
    Serial.write(Serial.read());
  } 
}

void HeartBeat(){
  if((millis() - beatTime) > 3000){
    //is servos not a t defaualt position, 
    if(!isAtDefault){
      isAtDefault = 1;
      int UpArmAngle = UpArmServo.DefaultPos;
      int LowArmAngle = LowArmServo.DefaultPos;
      int BaseAngle = BaseServo.DefaultPos;
      int WristPitchAngle = WristPitchServo.DefaultPos;
      int WristRollAngle = WristRollServo.DefaultPos;
      BaseServo.ServoSetPos((int)BaseAngle);
      LowArmServo.ServoSetPos((int)LowArmAngle);
      UpArmServo.ServoSetPos((int)UpArmAngle);
      WristPitchServo.ServoSetPos(WristPitchAngle);
      WristRollServo.ServoSetPos(WristRollAngle);
    }
    //Move the servo slightly
    BaseServo.ServoSetPos(BaseServo.DefaultPos - 1);
    //LowArmServo.ServoSetPos(LowArmServo.DefaultPos + 2);
    //UpArmServo.ServoSetPos(UpArmServo.DefaultPos + 2);
    delay(300);
    BaseServo.ServoSetPos(BaseServo.DefaultPos);    
    //LowArmServo.ServoSetPos(LowArmServo.DefaultPos - 2);
    //UpArmServo.ServoSetPos(UpArmServo.DefaultPos - 2);
    
    beatTime = millis();
  }
}