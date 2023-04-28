//           frames++;
//       if(millis() - ttime > 5000){
//         fps = frames*1.0/5.0;  
//         ttime = millis();          
//         frames = 0;
//       }

// float frames = 0;
// float fps = 0;
// unsigned long ttime = 0;

/*
  LCDEyes.ino - Simple Project that animates robot eyes on an LCD screen.
  Created by Nischay Joshi, 09-05-2023
*/

// Load TFT driver library
#include <TFT_eSPI.h>
#include <SPI.h>
#include "EyesClass.h"

TFT_eSPI tft = TFT_eSPI();  
EyesClass Eyes;
bool following = false;

unsigned long UARTReciveTime = 0; 

void setup()
{
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Serial Setup Done");
  randomSeed(analogRead(0));
  tft.init();            // initialize LCD
  tft.setRotation(1);
  //We want black background
  tft.fillScreen(BG_COLOR);
  Eyes.Initialize_Eyes();
  //Eyes.EyeTest(tft);
  UARTReciveTime = millis();
  Serial.println("Setup Routine Done");
  Eyes.WakeUp(tft);
  Eyes.LookAround(tft);
  //  while(1){
  //    Eyes.SadLookSide(tft);
  //  }
}


void loop()
{
  //Read Serial Input
  if(Serial.available()){
    UARTReciveTime = millis();
    //Read Serial Input
    String SerialData = Serial.readStringUntil('\n');
    //Echo Serial Input
    Serial.println(SerialData);
    
    //Now check if the Serial Input is a "FF, X,Y" string
    if(SerialData.substring(0, SerialData.indexOf(',')) == "FF"){
      SerialData = SerialData.substring(SerialData.indexOf(',') + 1);
      //Get the X and Y values
      int X = SerialData.substring(0, SerialData.indexOf(',')).toInt();
      int Y = SerialData.substring(SerialData.indexOf(',') + 1).toInt();

      //get the Final X and Y values
      int Eye1FinalX = map(X, 0, 255, 15, 75);
      int Eye1FinalY = map(Y, 0, 255, 24, 104);
      int Eye2FinalX = map(X, 0, 255, 85, 145);
      int Eye2FinalY = map(Y, 0, 255, 24, 104);

      //Set the X and Y values
      if(!following){
        Eyes.WakeUp(tft);
        following = true;
      }
      int Roll = random(0, 100);
      if(Roll < 35 && (millis() % 10000 < 1500)){
        Eyes.ChangeMoodRandom(tft);
        Eyes.Blink(tft);
      }     
      if(Roll < 50 && millis() % 10000 < 1000) {
        Eyes.Blink(tft);
      }       
      Eyes.Move_Eyes(tft, Eye1FinalX, Eye1FinalY, Eye2FinalX, Eye2FinalY, 4);
      //Clear the Serial Buffer
      while(Serial.available()){
        Serial.read();
      }
    }
  }
  if(millis() - UARTReciveTime > 5000){
    //Eyes.EyeTest(tft);
    following = false;
    Eyes.MoveEyesToCenter(tft);
    Eyes.DoSomething(tft);
    delay(500);
  }
  if(millis() - UARTReciveTime > 3000 && following == true){
    //Look around
    Eyes.MoveEyesToCenter(tft);    
    Eyes.LookSide(tft);
    delay(500);
    Eyes.SadLookSide(tft);
  }
  
}


