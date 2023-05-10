/*
  5 Axis Robotic Arm
  by: Nischay Joshi
  Date: 05/04/2023
  Description: This is a Controller for 6 axis robotic arm. Currently controlled by Serial Input
               The Arm is from this kit (https://a.co/d/7kXuqwg) and is slight modified to add one more axis of rotation. i.e 7 servos to control (including griper)
               ANG, 95, 50, 137, 100, 82, 85, 50, END               
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
ServoLib WristYawServo(WRIST_Y_SERVO_PIN, WRIST_Y_SERVO_MAX, WRIST_Y_SERVO_MIN, WRIST_Y_SERVO_SPEED, WRIST_Y_SERVO_DEF, WRIST_Y_SRVO_LOW_WIDTH, WRIST_Y_SRVO_HIGH_WIDTH);
ServoLib GripperServo(GRIPPER_SERVO_PIN, GRIPPER_SERVO_MAX, GRIPPER_SERVO_MIN, GRIPPER_SERVO_SPEED, GRIPPER_SERVO_OPEN, GRIPPER_SRVO_LOW_WIDTH, GRIPPER_SRVO_HIGH_WIDTH);


//Create the Controller Object
Controller RobotArmInput;
#define StickDeadZone 40
#define BaseTurnSpeed 0.05
#define LowArmTurnSpeed 0.02
#define UpArmTurnSpeed 0.02
#define WristPitchTurnSpeed 0.04
#define WristRollTurnSpeed 0.04
#define WristYawTurnSpeed 0.04

//Global Variables
#define NumServos 7
float BaseAngle;
float LowArmAngle;
float UpArmAngle;
float WristPitchAngle;
float WristRollAngle;
float WristYawAngle;
float GripperAngle;
float alpha = 0.04; // Low-pass filter weight, adjust this value as needed
float prev_output[NumServos] = {0}; // Previous output values for each servo

bool GripperState;  //0 = Close, 1 = Open
bool isAtDefault = false;

unsigned long updatedTime = 0;
unsigned long inputTime = 0;
unsigned long beatTime = 0;
unsigned long gripperTime = 0;

void ECHOSerialData();
void HeartBeat();
void GamepadControl();
void DebugStatements();
void AngleInputControl();
int CheckArrayEqual(int *array1, int* array2, int size);
float lowPassFilter(float input, int index);

void setup() {
  Serial.begin(115200);
  //Dont continue until Serial Comminication is established
  while (!Serial)
    ;

  //Setup the Servos
  BaseServo.SetupServo();
  LowArmServo.SetupServo();
  UpArmServo.SetupServo();
  WristPitchServo.SetupServo();
  WristRollServo.SetupServo();
  WristYawServo.SetupServo();
  GripperServo.SetupServo();

  //Intialize the Servo Angles
  BaseAngle = BaseServo.DefaultPos;
  LowArmAngle = LowArmServo.DefaultPos;
  UpArmAngle = UpArmServo.DefaultPos;
  WristPitchAngle = WristPitchServo.DefaultPos;
  WristRollAngle = WristRollServo.DefaultPos;
  WristYawAngle = WristYawServo.DefaultPos;
  GripperState = 0;

  //Send the Servo Angles
  BaseServo.ServoSetPos((int)BaseAngle);
  LowArmServo.ServoSetPos((int)LowArmAngle);
  UpArmServo.ServoSetPos((int)UpArmAngle);
  WristPitchServo.ServoSetPos((int)WristPitchAngle);
  WristRollServo.ServoSetPos((int)WristRollAngle);
  WristYawServo.ServoSetPos((int)WristYawAngle);
  GripperServo.ServoGoto(GRIPPER_SERVO_OPEN);
  isAtDefault = true;

  //Here we will make sure that the RobotArmInput is setup and ready to go
  if (!RobotArmInput.SetupController()) {
    Serial.println("Controller Setup Failed");
  }
  else{
    RobotArmInput.UpdateDataAverage();
  }
  Serial.println("Setup Complete");
  delay(1000);
}

void loop() {

  AngleInputControl();
  //GamepadControl();
  if (millis() - inputTime > 1000) {
    //HeartBeat();
  }
  delay(LOOP_DELAY);
}

void AngleInputControl(){
    int numServos = 7;
    int Angles[numServos] = {0};
    int CurrentAngles[numServos] = {0};

    if(Serial.available()){
      //Read the Serial Data
      String SerialData = Serial.readStringUntil('\n');
      //Now check if the Data's first item is the header
      if(SerialData.substring(0, SerialData.indexOf(',')) == "ANG"){
          SerialData = SerialData.substring(SerialData.indexOf(',') + 1);
          //Now we need to get the data
          for(int i = 0; i < numServos; i++){
              int index = SerialData.indexOf(",");
              Angles[i] = atoi(SerialData.substring(0, index).c_str());
              SerialData = SerialData.substring(index + 1);
          }
          //If the next element in the string left is 'END' then we can move the servos otheerwise the data is invalid and return
          if(!(SerialData.substring(0, SerialData.indexOf(',')) == " END")){
              Serial.println("Invalid Data");
              return;
          }

        //Get the servos current position
        CurrentAngles[0] = BaseServo.CurrentPos;
        CurrentAngles[1] = LowArmServo.CurrentPos;
        CurrentAngles[2] = UpArmServo.CurrentPos;
        CurrentAngles[3] = WristPitchServo.CurrentPos;
        CurrentAngles[4] = WristRollServo.CurrentPos;
        CurrentAngles[5] = WristYawServo.CurrentPos;
        CurrentAngles[6] = GripperServo.CurrentPos;
          int j = 0;
        //now we gradually move the servos to the new position
        while(!CheckArrayEqual(CurrentAngles, Angles, numServos)){
        //Update the angles
        for(int i = 0; i < numServos; i++){
          //Serial.print("Index: ");
          //Serial.println(i);
          if(Angles[i] != CurrentAngles[i]){
            // if(Angles[i] > CurrentAngles[i]){
            //   CurrentAngles[i] = round(lowPassFilter(CurrentAngles[i] + 1, i));
            // }
            // else{
            //   CurrentAngles[i] = round(lowPassFilter(CurrentAngles[i] - 1, i));
            // }
            CurrentAngles[i] = round(lowPassFilter(Angles[i], i));
          }
          //Serial.println(CurrentAngles[i]);
        }
        //Now Write the angles to the servo:
        BaseAngle = round(CurrentAngles[0]);
        LowArmAngle = round(CurrentAngles[1]);
        UpArmAngle = round(CurrentAngles[2]);
        WristPitchAngle = round(CurrentAngles[3]);
        WristRollAngle = round(CurrentAngles[4]);
        WristYawAngle = round(CurrentAngles[5]);
        GripperAngle = round(CurrentAngles[6]);
          //Send the Servo Angles
          BaseServo.ServoSetPos((int)BaseAngle);
          LowArmServo.ServoSetPos((int)LowArmAngle);
          UpArmServo.ServoSetPos((int)UpArmAngle);
          WristPitchServo.ServoSetPos((int)WristPitchAngle);
          WristRollServo.ServoSetPos((int)WristRollAngle);
          WristYawServo.ServoSetPos((int)WristYawAngle);
          GripperServo.ServoSetPos((int)GripperAngle);
          delay(20);
          j++;
          //DebugStatements();
        }
        //Serial.print("Filter iteration: ");
        //Serial.print(j);

          // BaseAngle = Angles[0];
          // LowArmAngle = Angles[1];
          // UpArmAngle = Angles[2];
          // WristPitchAngle = Angles[3];
          // WristRollAngle = Angles[4];
          // WristYawAngle = Angles[5];
          // GripperAngle = Angles[6];

          // BaseServo.ServoSetPos((int)BaseAngle);
          // LowArmServo.ServoSetPos((int)LowArmAngle);
          // UpArmServo.ServoSetPos((int)UpArmAngle);
          // WristPitchServo.ServoSetPos((int)WristPitchAngle);
          // WristRollServo.ServoSetPos((int)WristRollAngle);
          // WristYawServo.ServoSetPos((int)WristYawAngle);
          // GripperServo.ServoSetPos((int)GripperAngle);



        //  BaseServo.ServoGoto((int)BaseAngle);
        // LowArmServo.ServoGoto((int)LowArmAngle);
        // UpArmServo.ServoGoto((int)UpArmAngle);
        // WristPitchServo.ServoGoto((int)WristPitchAngle);
        // WristRollServo.ServoGoto((int)WristRollAngle);
        // WristYawServo.ServoGoto((int)WristYawAngle);
        // GripperServo.ServoSetPos((int)GripperAngle);

        //Print the angles
        //DebugStatements();
        Serial.println("0");
      }
    }     
}

void GamepadControl(){
  //Get the Controller Data
  if (RobotArmInput.GetControllerData()) {
    inputTime = millis();
    //Print the Controller Data
    RobotArmInput.PrintControllerData();
    //Set the Servo Angles
    BaseAngle += abs(RobotArmInput.DataAverage[0] - RobotArmInput.Data[0]) > StickDeadZone ? ((RobotArmInput.DataAverage[0] - RobotArmInput.Data[0]) * BaseTurnSpeed) : 0;
    LowArmAngle += abs(RobotArmInput.DataAverage[1] - RobotArmInput.Data[1]) > StickDeadZone ? ((RobotArmInput.DataAverage[1] - RobotArmInput.Data[1]) * LowArmTurnSpeed) : 0;
    int UpArm1 = RobotArmInput.Data[2] - RobotArmInput.DataAverage[2];
    int UpArm2 = RobotArmInput.Data[3] - RobotArmInput.DataAverage[3];
    UpArmAngle += abs(UpArm1) > StickDeadZone ? (UpArm1 * UpArmTurnSpeed) : 0;
    UpArmAngle -= abs(UpArm2) > StickDeadZone ? (UpArm2 * UpArmTurnSpeed) : 0;
    WristRollAngle += abs(RobotArmInput.DataAverage[9] - RobotArmInput.Data[9]) > StickDeadZone ? ((RobotArmInput.DataAverage[9] - RobotArmInput.Data[9]) * WristRollTurnSpeed * -1) : 0;
    WristPitchAngle += abs(RobotArmInput.DataAverage[10] - RobotArmInput.Data[10]) > StickDeadZone ? ((RobotArmInput.DataAverage[10] - RobotArmInput.Data[10]) * WristPitchTurnSpeed * -1) : 0;
    WristYawAngle += abs(RobotArmInput.DataAverage[11] - RobotArmInput.Data[11]) > StickDeadZone ? ((RobotArmInput.DataAverage[11] - RobotArmInput.Data[11]) * WristRollTurnSpeed * -1) : 0;

    //Check if the Angles are within the limits
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
    WristYawAngle = WristYawAngle < WRIST_Y_SERVO_MIN ? WRIST_Y_SERVO_MIN : WristYawAngle;

    if (RobotArmInput.Data[4] == 1 && GripperState == 1) {
      GripperState = 0;
      if (millis() - gripperTime > 1000) {
        gripperTime = millis();
        GripperServo.ServoGoto(GRIPPER_SERVO_CLOSE);
        Serial.println("Gripper Closed");
      }
    } else if (RobotArmInput.Data[4] == 1 && GripperState == 0) {
      GripperState = 1;
      if (millis() - gripperTime > 1000) {
        gripperTime = millis();
        GripperServo.ServoGoto(GRIPPER_SERVO_OPEN);
        Serial.println("Gripper Opened");
      }
    }

    //If Reset is pressed, reset the servos to their default positions
    if (RobotArmInput.Data[6]) {
      BaseAngle = BaseServo.DefaultPos;
    }
    if (RobotArmInput.Data[7]) {
      LowArmAngle = LowArmServo.DefaultPos;
    }
    if (RobotArmInput.Data[8]) {
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

}

void DebugStatements(){
  //Debug Statements
   Serial.println("------------------------------------");
   Serial.print("Base Angle Received: ");
   Serial.println(BaseAngle);
   Serial.print("Low Arm Angle Received: ");
   Serial.println(LowArmAngle);
   Serial.print("Up Arm Angle Received: ");
   Serial.println(UpArmAngle);
   Serial.print("Wrist Roll Angle Received: ");
   Serial.println(WristRollAngle);
   Serial.print("Wrist Pitch Angle Received: ");
   Serial.println(WristPitchAngle);
   Serial.print("Wrist Yaw Angle Received: ");
   Serial.println(WristYawAngle);
   Serial.print("Gripper Angle Received: ");
   Serial.println(GripperAngle);
   Serial.print("------------------------------------");
}

void ECHOSerialData() {
  if (Serial.available() > 0) {
    Serial.write(Serial.read());
  }
}

void HeartBeat() {
  if ((millis() - beatTime) > 3000) {
    //is servos not a t defaualt position,
    if (!isAtDefault) {
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

int CheckArrayEqual(int *array1, int* array2, int size){
  for(int i = 0; i < size; i++){
    if(array1[i] != array2[i]){
      return 0;
    }
  }
  return 1;
}

// Simple Low-Pass Filter
float lowPassFilter(float input, int index){
    //Serial.print("INPUT: ");
    //Serial.println(input);
    float output = alpha * input + (1.0 - alpha) * prev_output[index];
    prev_output[index] = output;
    //Serial.print("OUTPUT:");
    //Serial.println(output);
    return output;
}