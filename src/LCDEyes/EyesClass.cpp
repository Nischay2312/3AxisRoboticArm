#include "HardwareSerial.h"
/*
    *  EyesClass.cpp
    *  LCDEyes
    *
    *  By Nischay Joshi
    *   09-04-2023
*/

#include "EyesClass.h"

/*
    * Initialize the eyes
*/
void EyesClass::Initialize_Eyes(){
  //Initialize the eyes
  this->Eye1_centerX = 45;
  this->Eye1_centerY = 64;
  this->Eye2_centerX = 115;
  this->Eye2_centerY = 64;
  this->Eye1_Current_Width = EYE_WIDTH/2;
  this->Eye1_Current_Height = EYE_HEIGHT/2;
  this->Eye2_Current_Width = EYE_WIDTH/2;
  this->Eye2_Current_Height = EYE_HEIGHT/2;
  this->Eye_Color = EYE_COLOR;
  this->Iris_Color = IRIS_COLOR;
  this->Iris_radius = IRIS_RADIUS;
  this->Iris1_centerX = this->Eye1_centerX;
  this->Iris1_centerY = this->Eye1_centerY;
  this->Iris2_centerX = this->Eye2_centerX;
  this->Iris2_centerY = this->Eye2_centerY;
  this->Mood_State = Neutral;
}

/*
    * Function to test variety of effects
*/
void EyesClass::EyeTest(TFT_eSPI &tft){
/*

    //Clear eyes
    this->Clear_Eyes(tft);

    this->ChangeMood(Neutral);
    //First close both Eyes
    this->CloseEyes(tft);
    delay(100);
    //Now open the eyes
    this->OpenEyes(tft);
    delay(500);
    //Sqiunt the eyes -- slow
    this->Squint_Eyes(tft);
    //change mmood to Angry
    this->ChangeMood(Angry);
    delay(2000);
    //move the 2nd eye to the right
    this->Move_Eyes(tft, this->Eye1_centerX, this->Eye1_centerY, this->Eye2_centerX+3, this->Eye2_centerY, 2);
    delay(10);
    //Now close the eyes but not fully
    this->ShrinkEye(0, 12, 2, tft);
    delay(2000);
    //Now close completely
    this->ShrinkEye(0, 100, 2, tft);
    delay(100);
    //change mood to Sad
    this->ChangeMood(Happy);
    //Now open the eyes
    this->ShrinkEye(0, 60, 2, tft);
    //move the 2nd eye to the left
    this->Move_Eyes(tft, this->Eye1_centerX+10, this->Eye1_centerY+12, this->Eye2_centerX-1, this->Eye2_centerY+5, 3);
    delay(10);
    //now open it
    this->ShrinkEye(0, 40, 4, tft);
    delay(1000);
    //move eyes back to original position
    this->Move_Eyes(tft, this->Eye1_centerX-10, this->Eye1_centerY-12, this->Eye2_centerX-2, this->Eye2_centerY-5, 4);
    delay(200);

*/
    //this->Clear_Eyes(tft);
    //this->Wink(tft);
    //delay(500);
    //Set the robot to sleep
    this->Sleep(tft);
    delay(500);
    // //Wake up the robot
    this->WakeUp(tft);
    delay(500);
    //Wink
    this->Wink(tft);
    delay(500);
    //Close the eyes
    this->CloseEyes(tft);
    //Change mood randomly
    this->ChangeMoodRandom(tft);
    //this->ChangeMood(Happy);
    delay(500);
    //Open the eyes
    this->OpenEyes(tft);
    delay(150);
    this->Wink(tft);
    delay(500);
    //Squint the eyess
    this->Squint_Eyes(tft);
    delay(500);
    //Open the eyes
    this->OpenEyes(tft);  
    delay(250); 
    //wiggle the eyes
    this->EyeWiggle(tft);
    delay(500);
    //the rock effect
    this->TheRockLook(tft);
    //Sad look side
    this->SadLookSide(tft);
    delay(500);
}

/*
    * Function to Set the mood of the eyes
    * @param mood - Mood to set the eyes to
*/
void EyesClass::ChangeMood(Mood mood){
  this->Mood_State = mood;
}
/*
    * Function to animate chagnging the size of Eye.
    * @param EyeSelect - 1 for left eye, 2 for right eye, 0 for both eyes
    * @param Redction_pct - Percentage of eye closed
    * @param Frames - Number of frames to animate the change
    * @param tft - TFT_eSPI object to draw on
*/
void EyesClass::ShrinkEye(uint8_t EyeSelect, int16_t Redction_pct, uint16_t Frames, TFT_eSPI &tft){
    //Get the current height
    float Eye1_Current_Height = this->Eye1_Current_Height;
    float Eye2_Current_Height = this->Eye2_Current_Height;
    
    //Get the final height from EYE_HEIGHT and Redction_pct
    float Eye1_Final_Height = (EYE_HEIGHT/2 * (100-Redction_pct)) / 100;
    float Eye2_Final_Height = (EYE_HEIGHT/2 * (100-Redction_pct)) / 100;
    
    //Calculate the reduction in size per-frame
    float Eye1_Reduction = (Eye1_Current_Height - Eye1_Final_Height) / Frames;
    float Eye2_Reduction = (Eye2_Current_Height - Eye2_Final_Height) / Frames;
    
    //iterate per frame and draw the eyes
    for(int i = 0; i < Frames; i++){
        //Clear the old eyes
        this->Clear_Eyes(tft);

        //calculate the new height
        Eye1_Current_Height -= Eye1_Reduction;
        Eye2_Current_Height -= Eye2_Reduction;

        //update the current height
        this->Eye1_Current_Height = (EyeSelect == 1 || EyeSelect == 0)?floor(Eye1_Current_Height): this->Eye1_Current_Height;
        this->Eye2_Current_Height = (EyeSelect == 2 || EyeSelect == 0)?floor(Eye2_Current_Height): this->Eye2_Current_Height;
        
        //Now draw the new eyes
        this->Draw_Eyes(tft);
        //delay
        delay(50);
    }
}

/*
    * Clear the eye place
    * @param tft - TFT_eSPI object to draw on
*/
void EyesClass::Clear_Eyes(TFT_eSPI &tft){
    if(RECTANGULAR_EYES){
        //Clear the old eyes
        tft.fillRect(this->Eye1_centerX - this->Eye1_Current_Width-EYECLEAR_EXTRA, this->Eye1_centerY - this->Eye1_Current_Height-EYECLEAR_EXTRA, this->Eye1_Current_Width * 2+EYECLEAR_EXTRA*2, this->Eye1_Current_Height * 2+EYECLEAR_EXTRA*2, BG_COLOR);
        tft.fillRect(this->Eye2_centerX - this->Eye2_Current_Width-EYECLEAR_EXTRA, this->Eye2_centerY - this->Eye2_Current_Height-EYECLEAR_EXTRA, this->Eye2_Current_Width * 2+EYECLEAR_EXTRA*2, this->Eye2_Current_Height * 2+EYECLEAR_EXTRA*2, BG_COLOR);

    }
    else{
        //Clear the old eyes
        tft.fillEllipse(this->Eye1_centerX, this->Eye1_centerY, this->Eye1_Current_Width, this->Eye1_Current_Height, BG_COLOR);
        tft.fillEllipse(this->Eye2_centerX, this->Eye2_centerY, this->Eye2_Current_Width, this->Eye2_Current_Height, BG_COLOR);
    }
}

/*
    * Clear the eye place
    * @param tft - TFT_eSPI object to draw on
    * @param clear_all - If true, then clear the entire EYE_AREA
*/
void EyesClass::Clear_Eyes(TFT_eSPI &tft, bool clear_all){
    if(clear_all){
        tft.fillRect(this->Eye1_centerX - EYE_WIDTH/2, this->Eye1_centerY - EYE_HEIGHT, EYE_WIDTH, EYE_HEIGHT, BG_COLOR);
        tft.fillRect(this->Eye2_centerX - EYE_WIDTH/2, this->Eye2_centerY - EYE_HEIGHT, EYE_WIDTH, EYE_HEIGHT, BG_COLOR);
    }
    else{
        this->Clear_Eyes(tft);
    }
}

/*
    * Draw the eyes
    * @param tft - TFT_eSPI object to draw on
*/
void EyesClass::Draw_Eyes(TFT_eSPI &tft){
  tft.startWrite();
  
  //If the Eye width/hieghts are 0, then just make a black box to clear the eyes
  if(this->Eye1_Current_Width == 0 || this->Eye1_Current_Height == 0){
      tft.fillRect(this->Eye1_centerX - EYE_WIDTH/2, this->Eye1_centerY - EYE_HEIGHT, EYE_WIDTH, EYE_HEIGHT, BG_COLOR);
  }
  else{
    if(RECTANGULAR_EYES){
    //Clear the old eyes
        tft.fillRect(this->Eye1_centerX - this->Eye1_Current_Width, this->Eye1_centerY - this->Eye1_Current_Height, this->Eye1_Current_Width * 2, this->Eye1_Current_Height * 2, this->Eye_Color);
    }
    else{
        //Clear the old eyes
        tft.fillEllipse(this->Eye1_centerX, this->Eye1_centerY, this->Eye1_Current_Width, this->Eye1_Current_Height, this->Eye_Color);
    }
    //Draw the iris 1
    if(DRAWIRIS){
        tft.fillCircle(this->Iris1_centerX, this->Iris1_centerY, this->Iris_radius, this->Iris_Color);
    }
    //Draw the expression based on the mood state
    switch(this->Mood_State){
        case Happy:
            if(HAPPY_EYE_TYPE){
                //First cut the bottom of the eye
                tft.fillRect(this->Eye1_centerX - this->Eye1_Current_Width, this->Eye1_centerY, this->Eye1_Current_Width*2, this->Eye1_Current_Height, BG_COLOR);
                //Draw the happy expression
                tft.fillRoundRect(this->Eye1_centerX - this->Eye1_Current_Width, this->Eye1_centerY - this->Eye1_Current_Height - HAPPY_EYE_RADIUS, this->Eye1_Current_Width*2, this->Eye1_Current_Height/2, HAPPY_EYE_RADIUS, this->Eye_Color);
                //make a black box to clear the bottom of the eye
                tft.fillEllipse(this->Eye1_centerX-HAPPY_EYE_RADIUS, this->Eye1_centerY+this->Eye1_Current_Height/2, this->Eye1_Current_Width, this->Eye1_Current_Height+3, BG_COLOR);
            }
            else{
                //First cut the bottom of the eye
                tft.fillRect(this->Eye1_centerX - this->Eye1_Current_Width, this->Eye1_centerY, this->Eye1_Current_Width*2, this->Eye1_Current_Height, BG_COLOR);
                //Draw a trangle in the middle of the remaining top part of EYE.
                tft.fillTriangle(this->Eye1_centerX - this->Eye1_Current_Width*2/3, this->Eye1_centerY, this->Eye1_centerX, this->Eye1_centerY - this->Eye1_Current_Height*2/3, this->Eye1_centerX + this->Eye1_Current_Width*2/3, this->Eye1_centerY, BG_COLOR);
                //Now draw two traingles of BG color at the top left and right corners
                tft.fillTriangle(this->Eye1_centerX - this->Eye1_Current_Width, this->Eye1_centerY, this->Eye1_centerX - this->Eye1_Current_Width, this->Eye1_centerY - this->Eye1_Current_Height, this->Eye1_centerX, this->Eye1_centerY - this->Eye1_Current_Height, BG_COLOR);
                tft.fillTriangle(this->Eye1_centerX + this->Eye1_Current_Width, this->Eye1_centerY, this->Eye1_centerX + this->Eye1_Current_Width, this->Eye1_centerY - this->Eye1_Current_Height, this->Eye1_centerX, this->Eye1_centerY - this->Eye1_Current_Height, BG_COLOR);
            }
            break;
        case Sad:
            //Draw the sad expression, a simple inverted triangle
            tft.fillTriangle(this->Eye1_centerX - this->Eye1_Current_Width, this->Eye1_centerY, this->Eye1_centerX - this->Eye1_Current_Width, this->Eye1_centerY - this->Eye1_Current_Height, this->Eye1_centerX + this->Eye1_Current_Width, this->Eye1_centerY - this->Eye1_Current_Height, BG_COLOR);
            break;
        case Angry:
            //This is simple, make an inverted triangle starting on the top left corner
            tft.fillTriangle(this->Eye1_centerX - this->Eye1_Current_Width, this->Eye1_centerY - this->Eye1_Current_Height, this->Eye1_centerX + this->Eye1_Current_Width, this->Eye1_centerY - this->Eye1_Current_Height, this->Eye1_centerX + this->Eye1_Current_Width, this->Eye1_centerY, BG_COLOR);
            break;
        case Sleeping:
            //If the robot is sleeping then draw two super thin rectangles at each eye
            tft.fillRect(this->Eye1_centerX - this->Eye1_Current_Width, this->Eye1_centerY - this->Eye1_Current_Height, this->Eye1_Current_Width*2, 3, this->Eye_Color);             
        default:
            break;
    }
  }
  
  if(this->Eye2_Current_Width == 0 || this->Eye2_Current_Height == 0){
      tft.fillRect(this->Eye2_centerX - EYE_WIDTH/2, this->Eye2_centerY - EYE_HEIGHT, EYE_WIDTH, EYE_HEIGHT, BG_COLOR);
  }
  else{
    if(RECTANGULAR_EYES){
    //Clear the old eyes
        tft.fillRect(this->Eye2_centerX - this->Eye2_Current_Width, this->Eye2_centerY - this->Eye2_Current_Height, this->Eye2_Current_Width * 2, this->Eye2_Current_Height * 2, this->Eye_Color);
    }
    else{
        //Clear the old eyes
        tft.fillEllipse(this->Eye2_centerX, this->Eye2_centerY, this->Eye2_Current_Width, this->Eye2_Current_Height, this->Eye_Color);
    }
    //Draw the iris 2
    if(DRAWIRIS){
        tft.fillCircle(this->Iris2_centerX, this->Iris2_centerY, this->Iris_radius, this->Iris_Color);
    }
    
    //Draw the expression based on the mood state
    switch(this->Mood_State){
        case Happy:
            if(HAPPY_EYE_TYPE){
                //First cut the bottom of the eye
                tft.fillRect(this->Eye2_centerX - this->Eye2_Current_Width, this->Eye2_centerY, this->Eye2_Current_Width*2, this->Eye2_Current_Height, BG_COLOR);
                //Draw the happy expression
                tft.fillRoundRect(this->Eye2_centerX - this->Eye2_Current_Width, this->Eye2_centerY - this->Eye2_Current_Height - HAPPY_EYE_RADIUS, this->Eye2_Current_Width*2, this->Eye2_Current_Height/2, HAPPY_EYE_RADIUS, this->Eye_Color);
                //make a black box to clear the bottom of the eye
                tft.fillEllipse(this->Eye2_centerX+HAPPY_EYE_RADIUS, this->Eye2_centerY+this->Eye2_Current_Height/2, this->Eye2_Current_Width, this->Eye2_Current_Height+3, BG_COLOR);
            }
            else{
                //First cut the bottom of the eye
                tft.fillRect(this->Eye2_centerX - this->Eye2_Current_Width, this->Eye2_centerY, this->Eye2_Current_Width*2, this->Eye2_Current_Height, BG_COLOR);
                //Draw a trangle in the middle of the remaining top part of EYE.
                tft.fillTriangle(this->Eye2_centerX - this->Eye2_Current_Width*2/3, this->Eye2_centerY, this->Eye2_centerX, this->Eye2_centerY - this->Eye2_Current_Height*2/3, this->Eye2_centerX + this->Eye2_Current_Width*2/3, this->Eye2_centerY, BG_COLOR);
                //Now draw two traingles of BG color at the top left and right corners
                tft.fillTriangle(this->Eye2_centerX - this->Eye2_Current_Width, this->Eye2_centerY, this->Eye2_centerX - this->Eye2_Current_Width, this->Eye2_centerY - this->Eye2_Current_Height, this->Eye2_centerX, this->Eye2_centerY - this->Eye2_Current_Height, BG_COLOR);
                tft.fillTriangle(this->Eye2_centerX + this->Eye2_Current_Width, this->Eye2_centerY, this->Eye2_centerX + this->Eye2_Current_Width, this->Eye2_centerY - this->Eye2_Current_Height, this->Eye2_centerX, this->Eye2_centerY - this->Eye2_Current_Height, BG_COLOR);
            }
            break;
        case Sad:
            //Draw the sad expression
            tft.fillTriangle(this->Eye2_centerX + this->Eye2_Current_Width, this->Eye2_centerY, this->Eye2_centerX + this->Eye2_Current_Width, this->Eye2_centerY - this->Eye2_Current_Height, this->Eye2_centerX - this->Eye2_Current_Width, this->Eye2_centerY - this->Eye2_Current_Height, BG_COLOR);
            break;
        case Angry:
            //Same a eye 1 angry but sides flipped
            tft.fillTriangle(this->Eye2_centerX + this->Eye2_Current_Width, this->Eye2_centerY - this->Eye2_Current_Height, this->Eye2_centerX - this->Eye2_Current_Width, this->Eye2_centerY - this->Eye2_Current_Height, this->Eye2_centerX - this->Eye2_Current_Width, this->Eye2_centerY, BG_COLOR);
            break;
        case Sleeping:
            //If the robot is sleeping then draw two super thin rectangles at each eye, and some Z's on top of the right eye
            tft.fillRect(this->Eye2_centerX - this->Eye2_Current_Width, this->Eye2_centerY - this->Eye2_Current_Height, this->Eye2_Current_Width*2, 3, this->Eye_Color);

            switch (this->SleepAnimState){
                case 1:
                    tft.drawChar(this->Eye2_centerX + EYE_WIDTH/2, this->Eye2_centerY - EYE_HEIGHT/2, 'z', this->Eye_Color, BG_COLOR, 1);
                    this->SleepAnimState++;
                    break;
                case 2:
                    //Clean the last Z
                    tft.drawChar(this->Eye2_centerX + EYE_WIDTH/2+6, this->Eye2_centerY - EYE_HEIGHT/2 - 6, 'z', this->Eye_Color, BG_COLOR, 1);
                    this->SleepAnimState++;
                    break;
                case 3:
                    //Clean the last Z
                    tft.drawChar(this->Eye2_centerX + EYE_WIDTH/2+12, this->Eye2_centerY - EYE_HEIGHT/2 - 12, 'z', this->Eye_Color, BG_COLOR, 1);
                    this->SleepAnimState++;
                    break;
                case 0:
                    //Just clean all the Z
                    tft.fillRect(this->Eye2_centerX + EYE_WIDTH/2, this->Eye2_centerY - EYE_HEIGHT/2, 5, 7, BG_COLOR);
                    tft.fillRect(this->Eye2_centerX + EYE_WIDTH/2+6, this->Eye2_centerY - EYE_HEIGHT/2-6, 5, 7, BG_COLOR);
                    tft.fillRect(this->Eye2_centerX + EYE_WIDTH/2+12, this->Eye2_centerY - EYE_HEIGHT/2-12, 5, 7, BG_COLOR);
                    this->SleepAnimState = 1;
                    break;
                default:
                    this->SleepAnimState = 0;
            }
        default:
            break;
    }
  }
  //If in case the state is not sleeping and sleepanimstate is not 0 then reset it and clear the Z's
  if(this->Mood_State != Sleeping && this->SleepAnimState != 1){
        this->SleepAnimState = 1;
        //Just clean all the Z
        tft.fillRect(this->Eye2_centerX + EYE_WIDTH/2, this->Eye2_centerY - EYE_HEIGHT/2, 5, 7, BG_COLOR);
        tft.fillRect(this->Eye2_centerX + EYE_WIDTH/2+6, this->Eye2_centerY - EYE_HEIGHT/2-6, 5, 7, BG_COLOR);
        tft.fillRect(this->Eye2_centerX + EYE_WIDTH/2+12, this->Eye2_centerY - EYE_HEIGHT/2-12, 5, 7, BG_COLOR);
    }
  tft.endWrite();
}

/*
    * Move the eyes
    * @param Eye1_x - X coordinate of the first eye
    * @param Eye1_y - Y coordinate of the first eye
    * @param Eye2_x - X coordinate of the second eye
    * @param Eye2_y - Y coordinate of the second eye
*/
void EyesClass::Move_Eyes(int Eye1_x, int Eye1_y, int Eye2_x, int Eye2_y){
  //Move the eyes
  this->Eye1_centerX = Eye1_x;
  this->Eye1_centerY = Eye1_y;
  this->Eye2_centerX = Eye2_x;
  this->Eye2_centerY = Eye2_y;
}


/*
    *Function to Animate moving the eyes
    * @param tft - reference to the tft object
    * @param Eye1_x - Final X coordinate of the first eye
    * @param Eye1_y - Final Y coordinate of the first eye
    * @param Eye2_x - Final X coordinate of the second eye
    * @param Eye2_y - Final Y coordinate of the second eye
    * @param Frames - Number of frames to move the eyes in
*/
void EyesClass::Move_Eyes(TFT_eSPI &tft, int Eye1_x, int Eye1_y, int Eye2_x, int Eye2_y, uint16_t Frames){
    //Move the eyes
    float Eye1_X_Diff = (Eye1_x - this->Eye1_centerX)/(Frames-1);
    float Eye1_Y_Diff = (Eye1_y - this->Eye1_centerY)/(Frames-1);
    float Eye2_X_Diff = (Eye2_x - this->Eye2_centerX)/(Frames-1);
    float Eye2_Y_Diff = (Eye2_y - this->Eye2_centerY)/(Frames-1);
    for(int i = 0; i < Frames-1; i++){
        this->Clear_Eyes(tft);
        this->Move_Eyes(floor(this->Eye1_centerX + Eye1_X_Diff), floor(this->Eye1_centerY + Eye1_Y_Diff), floor(this->Eye2_centerX + Eye2_X_Diff), floor(this->Eye2_centerY + Eye2_Y_Diff));
        this->Draw_Eyes(tft);
        delay(ANIMATION_DELAY);
    }
    this->Move_Eyes(Eye1_x, Eye1_y, Eye2_x, Eye2_y);
    this->Draw_Eyes(tft);
    delay(ANIMATION_DELAY);
}

// Animation Functions
/*
    * Function to blink the eyes
    * @param tft - TFT_eSPI object to draw on
*/
void EyesClass::Blink(TFT_eSPI &tft){
    //close the eyes
    this->CloseEyes(tft);
    delay(180);
    //open the eyes
    this->OpenEyes(tft);
    delay(20);
}

/*
    * Function to wink the eyes
    * @param tft - TFT_eSPI object to draw on
*/
void EyesClass::Wink(TFT_eSPI &tft){
    //select a random eye to wink
    int Eye = random(1, 3);
    //shrink the eye
    this->ShrinkEye(Eye, 100, 3, tft);
    delay(20);
    //open the eye
    this->ShrinkEye(Eye, 0, 3, tft);
}

/*
    * Function to squint the eyes
    * @param tft - TFT_eSPI object to draw on
*/
void EyesClass::Squint_Eyes(TFT_eSPI &tft){
  this->ShrinkEye(0, 70, 6, tft);
}
/*
    * Function to close the eyes
    * @param tft - TFT_eSPI object to draw on
*/
void EyesClass::CloseEyes(TFT_eSPI &tft){
  this->ShrinkEye(0, 100, 3, tft);
}
/*
    * Function to open the eyes
    * @param tft - TFT_eSPI object to draw on
*/
void EyesClass::OpenEyes(TFT_eSPI &tft){
  this->ShrinkEye(0, 0, 3, tft);
}
/*
    * Function to randomly change the mood of the eyes, and then display the eyes.
    * @param tft - TFT_eSPI object to draw on
*/
void EyesClass::ChangeMoodRandom(TFT_eSPI &tft){
    //define the number of moods based on the length of the enum
    int MoodCount = this->Neutral - this->Happy + 1; 
    int mood = random(0, MoodCount);
    Mood Moodenum = static_cast<Mood>(Happy + mood);
    //change the mood
    this->ChangeMood(Moodenum);
    //Now display the eyes
    this->Draw_Eyes(tft);
}
/*
    * Function to make the robot go to sleep
    * @param tft - TFT_eSPI object to draw on
*/
void EyesClass::Sleep(TFT_eSPI &tft){
    //Slowly make the eyes smaller
    this->ShrinkEye(0, 50, 5, tft);
    this->ShrinkEye(0, 80, 8, tft);
    delay(100);
    this->ShrinkEye(0,90, 5, tft);
    //clear the eyes
    this->Clear_Eyes(tft, 1);
    this->ChangeMood(this->Sleeping);
    this->Draw_Eyes(tft);
    delay(150);
    this->Draw_Eyes(tft);
    delay(150);
    this->Draw_Eyes(tft);
    delay(150);
    this->Draw_Eyes(tft);
    delay(150);

}
/*
    * Function to make the robot wake up
    * @param tft - TFT_eSPI object to draw on
*/
void EyesClass::WakeUp(TFT_eSPI &tft){
    //Change the mood to neutral
    this->ChangeMood(this->Neutral);
    //Quickly Make the eyes bigger than normal
    this->ShrinkEye(0, -30, 3, tft);
    //delay(50);
    //Slowly make the eyes normal size
    this->ShrinkEye(0, 30, 6, tft);
    //Open the eye
    this->OpenEyes(tft);
    delay(50);
}

/*
    * Function to wiggle the eyes up and down
    * @param tft - TFT_eSPI object to draw on
*/
void EyesClass::EyeWiggle(TFT_eSPI &tft){
    //Make both Eyes wiggle
    //First make both eyes bigger
    this->ShrinkEye(0, -30, 3, tft);
    //return the eyes to normal size
    this->ShrinkEye(0, 0, 3, tft);
    //Make the eyes bigger again
    this->ShrinkEye(0, -30, 3, tft);
    //return the eyes to normal size
    this->ShrinkEye(0, 0, 3, tft);
}

/*
    * Function to animate a sad eye roll to the left
    * @param tft - TFT_eSPI object to draw on 
*/
void EyesClass::SadLookSide(TFT_eSPI &tft){
    //Change the mood to sad
    this->ChangeMood(this->Sad);
    //Store the current eye position
    int Eye1_x = this->Eye1_centerX;
    int Eye1_y = this->Eye1_centerY;
    int Eye2_x = this->Eye2_centerX;
    int Eye2_y = this->Eye2_centerY;
    //Move the eyes to the left
    this->Move_Eyes(tft, this->Eye1_centerX + 8, this->Eye1_centerY - 8, this->Eye2_centerX + 8, this->Eye2_centerY - 8, 5);
    this->Move_Eyes(tft, this->Eye1_centerX + 7, this->Eye1_centerY + 5, this->Eye2_centerX + 7, this->Eye2_centerY + 5, 4);
    this->Move_Eyes(tft, this->Eye1_centerX + 6, this->Eye1_centerY + 7, this->Eye2_centerX + 6, this->Eye2_centerY + 7, 4);
    this->Move_Eyes(tft, this->Eye1_centerX + 3, this->Eye1_centerY + 11, this->Eye2_centerX + 3, this->Eye2_centerY + 11, 3);
    //shrink the eyes
    this->ShrinkEye(0, 50, 6, tft);
    delay(1200);
    //Move the eyes back to the center
    this->Move_Eyes(tft, Eye1_x, Eye1_y, Eye2_x, Eye2_y, 4);
    this->ChangeMood(this->Neutral);
    this->Clear_Eyes(tft, 1);
    this->Draw_Eyes(tft);

}

//void CrazyLook(TFT_eSPI &tft);
/*
    * Function to make the robot Eyes make the Dwane Johnson "The Rock" look
    * @param tft - TFT_eSPI object to draw on
*/
void EyesClass::TheRockLook(TFT_eSPI &tft){
    //First make both eyes small about half and slowly
    //record the current mood
    int CurrentMood = this->Mood_State;
    Mood Moodenum = static_cast<Mood>(Happy + CurrentMood);
    //Change mood to angry
    this->ChangeMood(this->Angry);
    this->Draw_Eyes(tft);
    delay(500);
    this->ShrinkEye(0, 50, 12, tft);
    //Now make only one Eye Big
    int Eye = random(1, 3);
    this->ShrinkEye(Eye, -40, 4, tft);
    delay(800);
    //Revert to the previous mood
    this->ShrinkEye(0, 0, 4, tft);
    delay(500);
    this->ChangeMood(Moodenum);
}

/*
    * Function to call a random function from the list of functions
    * @param tft - TFT_eSPI object to draw on
*/
void EyesClass::DoSomething(TFT_eSPI &tft){
  
  //check if the eyes is sleeping
    if(this->Mood_State == this->Sleeping){
        //if the eyes are sleeping then wake them up
        //select randomly to either continue sleeping or wake up
        int WakeUp = random(0, 2);  
        if(WakeUp == 1){
            //wake up
            this->WakeUp(tft);
        }
        else{
            //continue sleeping
            this->Draw_Eyes(tft);        
        }
        return;
    }
        //List of functions to choose from
  
  //Functions to choose from
    void (EyesClass::*Functions[])(TFT_eSPI &tft) = {
        &EyesClass::Blink,
        &EyesClass::Wink,
        &EyesClass::Squint_Eyes,
        &EyesClass::CloseEyes,
        &EyesClass::OpenEyes,
        &EyesClass::ChangeMoodRandom,
        &EyesClass::Sleep,
        &EyesClass::WakeUp,
        &EyesClass::EyeWiggle,
        &EyesClass::SadLookSide,
        // void CrazyLook(TFT_eSPI &tft);
        &EyesClass::TheRockLook,
        };

    //Choose a random function, get the size of the array and then get a random number between 0 and the size of the array
    int Function = random(0, sizeof(Functions)/sizeof(Functions[0]));
    //Call the function
    (this->*Functions[Function])(tft);
}



/*
        ////////////////OLD CODE NOT USED/////////////////////
    //Draw the 1st iris 
    if(this->Eye1_Current_Height < this->Iris_radius && this->Eye1_Current_Width < this->Iris_radius){ //check is both height and width are less than the radius
        tft.fillCircle(this->Iris1_centerX, this->Iris1_centerY, this->Iris_radius, this->Iris_Color);
        //now cover the exceeding part with background color
        tft.fillRect(this->Iris1_centerX - this->Iris_radius, this->Iris1_centerY - this->Iris_radius, this->Iris_radius - this->Eye1_Current_Width, this->Iris_radius - this->Eye1_Current_Height, BG_COLOR);
    }
    else if(this->Eye1_Current_Height < this->Iris_radius){ //check if the height is less than the radius
        tft.fillCircle(this->Iris1_centerX, this->Iris1_centerY, this->Iris_radius, this->Iris_Color);
        //now cover the exceeding part with background color
        tft.fillRect(this->Iris1_centerX - this->Iris_radius, this->Iris1_centerY - this->Iris_radius, this->Iris_radius * 2, this->Iris_radius - this->Eye1_Current_Height, BG_COLOR);
        tft.fillRect(this->Iris1_centerX - this->Iris_radius, this->Iris1_centerY + this->Iris_radius, this->Iris_radius * 2, this->Iris_radius - this->Eye1_Current_Height, BG_COLOR);
    }
    else if(this->Eye1_Current_Width < this->Iris_radius){ //check if the width is less than the radius
        tft.fillCircle(this->Iris1_centerX, this->Iris1_centerY, this->Iris_radius, this->Iris_Color);
        //now cover the exceeding part with background color
        tft.fillRect(this->Iris1_centerX - this->Iris_radius, this->Iris1_centerY - this->Iris_radius, this->Iris_radius - this->Eye1_Current_Width, this->Iris_radius * 2, BG_COLOR);
    }
    else{   //if both height and width are greater than the radius
        tft.fillCircle(this->Iris1_centerX, this->Iris1_centerY, this->Iris_radius, this->Iris_Color);
    }

    //Draw the 2nd iris
    if(this->Eye2_Current_Height < this->Iris_radius && this->Eye2_Current_Width < this->Iris_radius){ //check is both height and width are less than the radius
        tft.fillCircle(this->Iris2_centerX, this->Iris2_centerY, this->Iris_radius, this->Iris_Color);
        //now cover the exceeding part with background color
        tft.fillRect(this->Iris2_centerX - this->Iris_radius, this->Iris2_centerY - this->Iris_radius, this->Iris_radius - this->Eye2_Current_Width, this->Iris_radius - this->Eye2_Current_Height, BG_COLOR);
    }
    else if(this->Eye2_Current_Height < this->Iris_radius){ //check if the height is less than the radius
        tft.fillCircle(this->Iris2_centerX, this->Iris2_centerY, this->Iris_radius, this->Iris_Color);
        //now cover the exceeding part with background color
        tft.fillRect(this->Iris2_centerX - this->Iris_radius, this->Iris2_centerY - this->Iris_radius, this->Iris_radius * 2, this->Iris_radius - this->Eye2_Current_Height, BG_COLOR);
        tft.fillRect(this->Iris2_centerX - this->Iris_radius, this->Iris2_centerY + this->Iris_radius, this->Iris_radius * 2, this->Iris_radius - this->Eye2_Current_Height, BG_COLOR);
    }
    else if(this->Eye2_Current_Width < this->Iris_radius){ //check if the width is less than the radius
        tft.fillCircle(this->Iris2_centerX, this->Iris2_centerY, this->Iris_radius, this->Iris_Color);
        //now cover the exceeding part with background color
        tft.fillRect(this->Iris2_centerX - this->Iris_radius, this->Iris2_centerY - this->Iris_radius, this->Iris_radius - this->Eye2_Current_Width, this->Iris_radius * 2, BG_COLOR);
    }
    else{   //if both height and width are greater than the radius
        tft.fillCircle(this->Iris2_centerX, this->Iris2_centerY, this->Iris_radius, this->Iris_Color);
    }    

*/