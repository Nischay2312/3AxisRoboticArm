/*
    *  EyesClass.h
    *  LCDEyes
    *
    *  By Nischay Joshi
    *  Created on 09-04-2023
*/

#ifndef EYESCLASS_H
#define EYESCLASS_H

#include <Arduino.h>
#include <math.h>
#include <TFT_eSPI.h>
#include <SPI.h>

#define EYE_WIDTH 30
#define EYE_HEIGHT 40
#define IRIS_RADIUS 3
#define BG_COLOR TFT_BLACK
#define EYE_COLOR TFT_WHITE
#define IRIS_COLOR TFT_BLUE
#define RECTANGULAR_EYES 1
#define DRAWIRIS 0
#define ANIMATION_DELAY 33  //ms

#define HAPPY_EYE_RADIUS 4
#define EYECLEAR_EXTRA 5


class EyesClass{
    private:
    enum Mood{
      Happy,
      Sad,
      Angry,
      Neutral
    };
    uint16_t Eye1_centerX;
    uint16_t Eye1_centerY;
    uint16_t Eye2_centerX;
    uint16_t Eye2_centerY;
    uint16_t Eye1_Current_Width;
    uint16_t Eye1_Current_Height;
    uint16_t Eye2_Current_Width;
    uint16_t Eye2_Current_Height;
    uint16_t Eye_Color;
    uint16_t Iris_Color;
    uint16_t Iris_radius;
    uint16_t Iris1_centerX;
    uint16_t Iris1_centerY;
    uint16_t Iris2_centerX;
    uint16_t Iris2_centerY;
    uint8_t Mood_State;
    
    public:
    void Initialize_Eyes();
    void Animate_Blink(TFT_eSPI &tft);
    void ShrinkEye(uint8_t EyeSelect, int16_t Redction_pct, uint16_t Frames, TFT_eSPI &tft);
    void Clear_Eyes(TFT_eSPI &tft);
    void Draw_Eyes(TFT_eSPI &tft);
    void Move_Eyes(int Eye1_x, int Eye1_y, int Eye2_x, int Eye2_y);
    void Move_Eyes(TFT_eSPI &tft, int Eye1_x, int Eye1_y, int Eye2_x, int Eye2_y, uint16_t Frames);
};

#endif