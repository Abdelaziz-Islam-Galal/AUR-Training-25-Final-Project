#ifndef ENCODER_H
#define ENCODER_H 

#include <Arduino.h>
#define ENCODER_LEFT_A 34
#define ENCODER_LEFT_B 35
#define ENCODER_RIGHT_A 36
#define ENCODER_RIGHT_B 39
void EncoderInit();
void updateEncoders();
float getCurrentAngle();

#endif